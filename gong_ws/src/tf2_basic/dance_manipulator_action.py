import os
import random
import yaml
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import (
    FollowJointTrajectory,
    FollowJointTrajectory_GetResult_Response,
    GripperCommand,
    GripperCommand_GetResult_Response,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class DanceManipulatorAction(Node):
    def __init__(self):
        super().__init__("dance_manipulator_action")  # 노드 이름 설정

        # 하드웨어 및 충돌 방지를 위한 소프트웨어 조인트 안전 한계치 (라디안 단위)
        self.joint_limits = [
            {"min": -0.8, "max": 0.8},  # joint1
            {"min": -0.8, "max": 0.8},  # joint2
            {"min": -0.8, "max": 0.8},  # joint3
            {"min": -0.8, "max": 0.8},  # joint4
        ]

        # 기본 백업 포즈 (YAML 로드 실패 시 적용되는 안전용 예비 데이터)
        self.pose_list = [
            [0.2, -0.3, 0.2, -0.2],
            [-0.2, -0.4, 0.3, 0.2]
        ]
        
        # 외부 YAML 파일 로드 함수 호출
        self.load_position_data()

        # [토픽 -> 액션 변경] 관절 제어를 위한 액션 클라이언트 등록
        self.joint_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        # 그리퍼 제어를 위한 액션 클라이언트 등록
        self.gripper_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        
        # 현재 로봇 상태 모니터링을 위한 서브스크립션
        self.joint_state_subscription = self.create_subscription(
            JointState, "joint_states", self.joint_callback, 10
        )
        
        self.current_joint_position = [0.0, 0.0, 0.0, 0.0]
        self.duration_sec = 1.5  # 목표 지점 도달 시간 (초 단위)

        # 2초 간격으로 랜덤 제어를 수행하는 타이머 등록
        self.create_timer(2.0, self.timer_callback)
        self.get_logger().info("액션 기반 안전 랜덤 댄스 노드가 시작되었습니다.")

    def load_position_data(self):
        """스크립트 절대 경로를 기준으로 positions.yaml 파일을 로드합니다."""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        yaml_path = os.path.join(current_dir, "positions.yaml")
        
        if os.path.exists(yaml_path):
            try:
                with open(yaml_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                    if data and "dance_poses" in data:
                        self.pose_list = data["dance_poses"]
                        self.get_logger().info(f"성공적으로 {len(self.pose_list)}개의 포즈 데이터를 파일에서 읽어왔습니다.")
            except Exception as e:
                self.get_logger().warn(f"YAML 파일 읽기 실패, 백업 데이터를 사용합니다: {e}")
        else:
            self.get_logger().warn(f"설정 파일({yaml_path})이 없습니다. 기본 백업 포즈로 동작합니다.")

    def timer_callback(self):
        """2초 주기로 무작위 관절 각도와 그리퍼 스텝을 실행하는 콜백"""
        if not self.pose_list:
            return

        # 1. random.choice를 이용한 무작위 댄스 포즈 선택
        random_pose = random.choice(self.pose_list)
        
        # 2. 소프트웨어 한계값(Soft Limit) 적용을 위한 강제 클리핑 안전조치
        safe_pose = []
        for i in range(min(len(random_pose), 4)):
            clipped_val = max(self.joint_limits[i]["min"], min(random_pose[i], self.joint_limits[i]["max"]))
            safe_pose.append(clipped_val)

        # 3. Trajectory 포인트 객체 생성 및 시간 할당
        point = JointTrajectoryPoint()
        point.positions = safe_pose
        seconds = int(self.duration_sec)
        nanoseconds = int((self.duration_sec - seconds) * 1_000_000_000)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds
        
        # 4. 그리퍼도 급격히 개폐되지 않도록 좁고 잔잔한 범위(0.012m ~ 0.018m) 안에서 랜덤 연산
        random_gripper_pos = random.uniform(0.012, 0.018)

        # 5. 로봇팔 조인트 및 그리퍼 비동기 액션 요청 동시 송신
        self.move_joint(point)
        self.move_gripper(random_gripper_pos)

    def joint_callback(self, msg: JointState):
        self.current_joint_position = msg.position

    # --- 그리퍼 액션 클라이언트 메서드 부 ---
    def move_gripper(self, position: float, max_effort=10.0, timeout_sec=1.0):
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        
        send_goal_future = self.gripper_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_callback)

    def goal_callback(self, future: Future):
        self.goal_handle = future.result()
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result: GripperCommand_GetResult_Response = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[그리퍼] 이동 완료: {result.result.position:.4f}m")

    # --- [수정 및 추가] 관절 trajectory 액션 클라이언트 메서드 부 ---
    def move_joint(self, point: JointTrajectoryPoint):
        if not self.joint_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("joint_controller 액션 서버가 열려있지 않습니다.")
            return
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.header.frame_id = "move_manipulator"
        goal.trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        goal.trajectory.points.append(point)

        send_goal_future = self.joint_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_joint_callback)

    def goal_joint_callback(self, future: Future):
        self.goal_handle = future.result()
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_joint_result_callback)

    def get_joint_result_callback(self, future: Future):
        result: FollowJointTrajectory_GetResult_Response = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            formatted_pose = ", ".join([f"{p:.3f}" for p in self.current_joint_position[:4]])
            self.get_logger().info(f"[로봇팔] 액션 목표 도달 성공! 현재 각도: [{formatted_pose}]")
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("[로봇팔] 액션 명령이 중단(Aborted)되었습니다.")


def main(args=None):
    rclpy.init(args=args)
    node = DanceManipulatorAction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("사용자 요청으로 노드를 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
# 로봇팔을 움직여서 춤추는 동작을 구현하시오.
# random함수를 활용
# position 정보는 data파일을 로드해서 구현(txt, yaml, sqlite ...)

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand, GripperCommand_GetResult_Response
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import random
import os
import yaml  # 외부 yaml 데이터를 파싱하기 위해 추가 [cite: 33]

class DanceManipulator(Node):
    def __init__(self):
        super().__init__("dance_manipulator")  # 노드 이름 설정
        
        # 1. 외부 YAML 파일에서 춤 포즈 데이터 로드 [cite: 31, 33]
        self.pose_list = self.load_position_data()
        
        # 2. 퍼블리셔, 액션 클라이언트, 서브스크립션 설정 [cite: 2, 11, 12, 13]
        self.pub = self.create_publisher(JointTrajectory, "arm_controller/joint_trajectory", 10) [cite: 2, 11]
        self.gripper_client = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd") [cite: 2, 12]
        self.joint_state_subscription = self.create_subscription(
            JointState, "joint_states", self.joint_callback, 10
        ) [cite: 2, 13]
        
        # 상태 변수 초기화 [cite: 2]
        self.current_joint_position = [0.0, 0.0, 0.0, 0.0] [cite: 2]
        self.duration_sec = 2.0 [cite: 2, 35]
        
        # 3. 타이머 등록 (2초마다 무작위 춤 동작 수행) [cite: 2, 35]
        self.create_timer(self.duration_sec, self.timer_callback) [cite: 2]

    def load_position_data(self):
        """외부 yaml 파일에서 로봇팔 춤 포즈 데이터를 로드하는 함수 [cite: 31, 33]"""
        file_path = os.path.join(os.getcwd(), "positions.yaml") [cite: 36]
        
        # 예외 처리: 파일이 존재하지 않을 경우 기본 백업 포즈 제공 [cite: 3, 4]
        if not os.path.exists(file_path):
            self.get_logger().warning(f"데이터 파일({file_path})을 찾을 수 없어 기본 포즈를 사용합니다.")
            return [
                [0.8995922516973869, -0.5832234352774157, -0.26300971181849175, -0.6258641614575486], [cite: 3]
                [0.4795922516973865, -0.883223435277416, 0.3969902881815083, 0.8941358385424522] [cite: 4]
            ]
            
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
                poses = data.get("dance_poses", [])
                self.get_logger().info(f"성공적으로 {len(poses)}개의 춤 포즈 데이터를 로드했습니다.")
                return poses
        except Exception as e:
            self.get_logger().error(f"파일을 읽는 중 오류가 발생했습니다: {e}")
            return [[0.0, 0.0, 0.0, 0.0]]

    def timer_callback(self):
        msg = JointTrajectory() [cite: 3]
        msg.header.stamp = self.get_clock().now().to_msg() [cite: 3]
        msg.header.frame_id = "move_manipulator" [cite: 3]
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4"] [cite: 3]
        
        point = JointTrajectoryPoint() [cite: 3]
        
        # 4. random 함수를 활용한 무작위 춤 포즈 결정 [cite: 31, 34]
        random_pose = random.choice(self.pose_list) [cite: 34]
        point.positions = [float(val) for val in random_pose]
        
        # 그리퍼의 벌림 정도(0.01m ~ 0.02m)도 무작위로 연산하여 전달 [cite: 3, 4, 34]
        random_gripper_pos = random.uniform(0.01, 0.02)
        self.move_gripper(random_gripper_pos) [cite: 3, 4]
        
        self.get_logger().info(f"💃 랜덤 춤 실행 중 -> 포즈: {point.positions} | 그리퍼: {random_gripper_pos:.3f}")

        # 시간 정보 계산 및 주입 [cite: 4, 5]
        seconds = int(self.duration_sec) [cite: 4]
        nanoseconds = int((self.duration_sec - seconds) * 1_000_000_000) [cite: 4]
        point.time_from_start.sec = seconds [cite: 5]
        point.time_from_start.nanosec = nanoseconds [cite: 5]

        msg.points.append(point)  # type: ignore [cite: 5]
        self.pub.publish(msg) [cite: 5]

    def joint_callback(self, msg: JointState):
        self.current_joint_position = msg.position [cite: 5]

    def move_gripper(self, position: float, max_effort=10.0, timeout_sec=5.0):
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec): [cite: 5]
            self.get_logger().info("gripper_controller Action 서버를 찾지 못했습니다.") [cite: 5]
            return
        goal = GripperCommand.Goal() [cite: 5]
        goal.command.position = float(position) [cite: 5]
        goal.command.max_effort = float(max_effort) [cite: 5]
        send_goal_future = self.gripper_client.send_goal_async(goal) [cite: 5]
        send_goal_future.add_done_callback(self.goal_callback) [cite: 5]

    def goal_callback(self, future: Future):
        self.goal_handle = future.result()  # type: ignore [cite: 6]
        self.get_result_future = self.goal_handle.get_result_async()  # type: ignore [cite: 6]
        self.get_result_future.add_done_callback(self.get_result_callback) [cite: 6]

    def feedback_callback(self, msg: GripperCommand.Impl.FeedbackMessage):
        feedback: GripperCommand.Feedback = msg.feedback [cite: 6]
        self.get_logger().info(f"{feedback.position}") [cite: 6]

    def get_result_callback(self, future: Future):
        result: GripperCommand_GetResult_Response = future.result()  # type: ignore [cite: 7]
        if result.status == GoalStatus.STATUS_SUCCEEDED: [cite: 7]
            pass  # 반복적인 춤 동작 콘솔창을 깨끗하게 유지하기 위해 로그 생략
        elif result.status == GoalStatus.STATUS_ABORTED: [cite: 7]
            self.get_logger().info("그리퍼 구동 실패(Aborted)!!") [cite: 7]
        elif result.status == GoalStatus.STATUS_CANCELED: [cite: 7]
            self.get_logger().info("그리퍼 구동 취소(Canceled)!!") [cite: 7]


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화 [cite: 7]
    node = DanceManipulator()
    try:
        rclpy.spin(node)  # 무한 루프 구동 [cite: 7]
    except KeyboardInterrupt:
        print("키보드 인터럽트로 춤 동작을 종료합니다.") [cite: 7]
    finally:
        node.destroy_node() [cite: 8]


if __name__ == "__main__":
    main() [cite: 9]
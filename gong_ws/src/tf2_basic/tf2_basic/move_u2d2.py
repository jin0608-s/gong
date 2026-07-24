# 팔을 추가해서 엘보우 구현(revolure) (다리 옆으로 막대기 2개 달면 됨)
# 05_add_arm.urdf
#실행 ros2 launch tf2_basic_display.launch.py model:=urdf/04_pysics.urdf gui:=true
# tf을 발행해서 머리를 돌리거나 막대기를 꺼나, 바퀴를 굴려보세요. 재미있게 동작하게
# ros2 run tf2_basic move_u2d2로 움직이기



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class MoveU2D2TF(Node):
    def __init__(self):
        super().__init__('move_u2d2')
        
        # TF를 발행하기 위한 Broadcaster 선언
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 0.03초(약 33Hz) 주기로 TF를 갱신하는 타이머
        self.timer = self.create_timer(0.03, self.timer_callback)
        
        self.time_counter = 0.0
        self.get_logger().info('===============================================')
        self.get_logger().info(' TF 직접 발행 방식의 U2D2 제어 노드가 시작되었습니다!')
        self.get_logger().info(' 조인트 상태가 아닌 동적 좌표계(TF) 변환으로 움직입니다. 🕺')
        self.get_logger().info('===============================================')

    # 오일러 각(Roll, Pitch, Yaw)을 TF에 필요한 Quaternion(x, y, z, w)으로 변환하는 함수
    def euler_to_quaternion(self, r, p, y):
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)

        q = [0.0, 0.0, 0.0, 1.0]
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        return q

    def timer_callback(self):
        self.time_counter += 0.03
        now = self.get_clock().now().to_msg()

        # 각 관절/링크의 재미있는 움직임 계산
        head_yaw = math.sin(self.time_counter * 3.0) * 1.2             # 머리 도리도리 (Yaw)
        gripper_x = 0.19 + (math.sin(self.time_counter * 2.5) - 1.0) * 0.19 # 막대기 앞뒤 찌르기 (X축 이동)
        wheel_pitch = self.time_counter * 5.0                          # 바퀴 회전 (Pitch)
        
        # 팔/엘보우용 흔들림 각도 계산
        r_shoulder_pitch = math.sin(self.time_counter * 4.0) * 1.0     # 우측 어깨 앞뒤 흔들기
        l_shoulder_pitch = -math.sin(self.time_counter * 4.0) * 1.0    # 좌측 어깨 반대로 흔들기
        r_elbow_pitch = (math.sin(self.time_counter * 5.0) + 1.0) * 1.0 # 우측 팔꿈치 굽히기
        l_elbow_pitch = (math.cos(self.time_counter * 5.0) + 1.0) * 1.0 # 좌측 팔꿈치 굽히기

        transforms = []

        # 1. 머리 회전 TF (base_link -> head)
        t_head = TransformStamped()
        t_head.header.stamp = now
        t_head.header.frame_id = 'base_link'
        t_head.child_frame_id = 'head'
        t_head.transform.translation.x = 0.0
        t_head.transform.translation.y = 0.0
        t_head.transform.translation.z = 0.3
        q_head = self.euler_to_quaternion(0.0, 0.0, head_yaw)
        t_head.transform.rotation.x = q_head[0]
        t_head.transform.rotation.y = q_head[1]
        t_head.transform.rotation.z = q_head[2]
        t_head.transform.rotation.w = q_head[3]
        transforms.append(t_head)

        # 2. 그리퍼 확장 막대기 TF (base_link -> gripper_pole)
        t_pole = TransformStamped()
        t_pole.header.stamp = now
        t_pole.header.frame_id = 'base_link'
        t_pole.child_frame_id = 'gripper_pole'
        t_pole.transform.translation.x = gripper_x  # 시간에 따라 X 변환값이 바뀜 (꺼내기)
        t_pole.transform.translation.y = 0.0
        t_pole.transform.translation.z = 0.2
        t_pole.transform.rotation.x = 0.0
        t_pole.transform.rotation.y = 0.0
        t_pole.transform.rotation.z = 0.0
        t_pole.transform.rotation.w = 1.0
        transforms.append(t_pole)

        # 3. 우측 바퀴들 회전 TF (right_base -> front/back wheel)
        for wheel_name, offset_x in [('right_front_wheel', 0.133), ('right_back_wheel', -0.133)]:
            t_wheel = TransformStamped()
            t_wheel.header.stamp = now
            t_wheel.header.frame_id = 'right_base'
            t_wheel.child_frame_id = wheel_name
            t_wheel.transform.translation.x = offset_x
            t_wheel.transform.translation.y = 0.0
            t_wheel.transform.translation.z = -0.085
            q_w = self.euler_to_quaternion(0.0, wheel_pitch, 0.0) # Y축(Pitch) 회전으로 굴리기
            t_wheel.transform.rotation.x = q_w[0]
            t_wheel.transform.rotation.y = q_w[1]
            t_wheel.transform.rotation.z = q_w[2]
            t_wheel.transform.rotation.w = q_w[3]
            transforms.append(t_wheel)

        # 4. 좌측 바퀴들 회전 TF (left_base -> front/back wheel)
        for wheel_name, offset_x in [('left_front_wheel', 0.133), ('left_back_wheel', -0.133)]:
            t_wheel = TransformStamped()
            t_wheel.header.stamp = now
            t_wheel.header.frame_id = 'left_base'
            t_wheel.child_frame_id = wheel_name
            t_wheel.transform.translation.x = offset_x
            t_wheel.transform.translation.y = 0.0
            t_wheel.transform.translation.z = -0.085
            q_w = self.euler_to_quaternion(0.0, wheel_pitch, 0.0)
            t_wheel.transform.rotation.x = q_w[0]
            t_wheel.transform.rotation.y = q_w[1]
            t_wheel.transform.rotation.z = q_w[2]
            t_wheel.transform.rotation.w = q_w[3]
            transforms.append(t_wheel)

        # 5. 우측 어깨 및 엘보우 TF
        # right_leg -> right_arm_upper
        t_r_shoulder = TransformStamped()
        t_r_shoulder.header.stamp = now
        t_r_shoulder.header.frame_id = 'right_leg'
        t_r_shoulder.child_frame_id = 'right_arm_upper'
        t_r_shoulder.transform.translation.x = 0.0
        t_r_shoulder.transform.translation.y = -0.08
        t_r_shoulder.transform.translation.z = -0.1
        q_rs = self.euler_to_quaternion(0.0, r_shoulder_pitch, 0.0)
        t_r_shoulder.transform.rotation.x = q_rs[0]
        t_r_shoulder.transform.rotation.y = q_rs[1]
        t_r_shoulder.transform.rotation.z = q_rs[2]
        t_r_shoulder.transform.rotation.w = q_rs[3]
        transforms.append(t_r_shoulder)

        # right_arm_upper -> right_arm_lower (엘보우)
        t_r_elbow = TransformStamped()
        t_r_elbow.header.stamp = now
        t_r_elbow.header.frame_id = 'right_arm_upper'
        t_r_elbow.child_frame_id = 'right_arm_lower'
        t_r_elbow.transform.translation.x = 0.0
        t_r_elbow.transform.translation.y = 0.0
        t_r_elbow.transform.translation.z = -0.25
        q_re = self.euler_to_quaternion(0.0, r_elbow_pitch, 0.0)
        t_r_elbow.transform.rotation.x = q_re[0]
        t_r_elbow.transform.rotation.y = q_re[1]
        t_r_elbow.transform.rotation.z = q_re[2]
        t_r_elbow.transform.rotation.w = q_re[3]
        transforms.append(t_r_elbow)

        # 6. 좌측 어깨 및 엘보우 TF
        # left_leg -> left_arm_upper
        t_l_shoulder = TransformStamped()
        t_l_shoulder.header.stamp = now
        t_l_shoulder.header.frame_id = 'left_leg'
        t_l_shoulder.child_frame_id = 'left_arm_upper'
        t_l_shoulder.transform.translation.x = 0.0
        t_l_shoulder.transform.translation.y = 0.08
        t_l_shoulder.transform.translation.z = -0.1
        q_ls = self.euler_to_quaternion(0.0, l_shoulder_pitch, 0.0)
        t_l_shoulder.transform.rotation.x = q_ls[0]
        t_l_shoulder.transform.rotation.y = q_ls[1]
        t_l_shoulder.transform.rotation.z = q_ls[2]
        t_l_shoulder.transform.rotation.w = q_ls[3]
        transforms.append(t_l_shoulder)

        # left_arm_upper -> left_arm_lower (엘보우)
        t_l_elbow = TransformStamped()
        t_l_elbow.header.stamp = now
        t_l_elbow.header.frame_id = 'left_arm_upper'
        t_l_elbow.child_frame_id = 'left_arm_lower'
        t_l_elbow.transform.translation.x = 0.0
        t_l_elbow.transform.translation.y = 0.0
        t_l_elbow.transform.translation.z = -0.25
        q_le = self.euler_to_quaternion(0.0, l_elbow_pitch, 0.0)
        t_l_elbow.transform.rotation.x = q_le[0]
        t_l_elbow.transform.rotation.y = q_le[1]
        t_l_elbow.transform.rotation.z = q_le[2]
        t_l_elbow.transform.rotation.w = q_le[3]
        transforms.append(t_l_elbow)

        # 모든 변화값(TF)들을 일괄 발행
        self.get_logger().debug('TF 브로드캐스팅 중...')
        self.tf_broadcaster.sendTransform(transforms)

def main(args=None):
    rclpy.init(args=args)
    node = MoveU2D2TF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
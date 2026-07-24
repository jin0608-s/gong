# 팔을 추가해서 엘보우 구현(revolure) (다리 옆으로 막대기 2개 달면 됨)
# 05_add_arm.urdf
#실행 ros2 launch tf2_basic_display.launch.py model:=urdf/04_pysics.urdf gui:=true
# tf을 발행해서 머리를 돌리거나 막대기를 꺼나, 바퀴를 굴려보세요. 재미있게 동작하게
# ros2 run tf2_basic move_u2d2로 움직이기




import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
import tf2_ros
import math

class MoveU2D2TF(Node):
    def __init__(self):
        super().__init__('move_u2d2')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 0.01초(100Hz) 주기로 변경하여 디스플레이 주사율과 완벽 동기화 (렉 제거 핵심)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.time_counter = 0.0
        self.get_logger().info('===============================================')
        self.get_logger().info(' 초고속 100Hz 모드 및 타임 버퍼가 적용되었습니다.')
        self.get_logger().info(' 이제 떨림과 끊김 현상 없이 부드럽게 동작합니다! 🕺')
        self.get_logger().info('===============================================')

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

    def make_transform(self, now, parent, child, x, y, z, q):
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t

    def timer_callback(self):
        # 100Hz 주기에 맞추어 카운터 누적값 조절 (속도가 너무 빨라지지 않게 보정)
        self.time_counter += 0.01
        
        # [핵심 튜닝] 연산 및 통신 지연으로 인한 RViz 끊김을 막기 위해 
        # 현재 시간에 0.02초(20ms)의 미세한 가상 버퍼 시간을 더해줌.
        now = (self.get_clock().now() + Duration(seconds=0.02)).to_msg()

        # 관절 곡선 연산
        head_yaw = math.sin(self.time_counter * 3.0) * 1.2
        gripper_x = 0.19 + (math.sin(self.time_counter * 2.5) - 1.0) * 0.19
        wheel_pitch = self.time_counter * 5.0
        
        r_shoulder_pitch = math.sin(self.time_counter * 4.0) * 1.0
        l_shoulder_pitch = -math.sin(self.time_counter * 4.0) * 1.0
        r_elbow_pitch = (math.sin(self.time_counter * 5.0) + 1.0) * 1.0
        l_elbow_pitch = (math.cos(self.time_counter * 5.0) + 1.0) * 1.0

        transforms = []

        # 1. 머리 회전
        transforms.append(self.make_transform(now, 'base_link', 'head', 0.0, 0.0, 0.3, 
                                              self.euler_to_quaternion(0.0, 0.0, head_yaw)))

        # 2. 그리퍼 확장 막대
        transforms.append(self.make_transform(now, 'base_link', 'gripper_pole', gripper_x, 0.0, 0.2, 
                                              [0.0, 0.0, 0.0, 1.0]))

        # 3. 우측 바퀴들
        q_w = self.euler_to_quaternion(0.0, wheel_pitch, 0.0)
        transforms.append(self.make_transform(now, 'right_base', 'right_front_wheel', 0.133, 0.0, -0.085, q_w))
        transforms.append(self.make_transform(now, 'right_base', 'right_back_wheel', -0.133, 0.0, -0.085, q_w))

        # 4. 좌측 바퀴들
        transforms.append(self.make_transform(now, 'left_base', 'left_front_wheel', 0.133, 0.0, -0.085, q_w))
        transforms.append(self.make_transform(now, 'left_base', 'left_back_wheel', -0.133, 0.0, -0.085, q_w))

        # 5. 우측 팔 시리즈
        transforms.append(self.make_transform(now, 'right_leg', 'right_arm_upper', 0.0, -0.08, -0.1, 
                                              self.euler_to_quaternion(0.0, r_shoulder_pitch, 0.0)))
        transforms.append(self.make_transform(now, 'right_arm_upper', 'right_arm_lower', 0.0, 0.0, -0.25, 
                                              self.euler_to_quaternion(0.0, r_elbow_pitch, 0.0)))

        # 6. 좌측 팔 시리즈
        transforms.append(self.make_transform(now, 'left_leg', 'left_arm_upper', 0.0, 0.08, -0.1, 
                                              self.euler_to_quaternion(0.0, l_shoulder_pitch, 0.0)))
        transforms.append(self.make_transform(now, 'left_arm_upper', 'left_arm_lower', 0.0, 0.0, -0.25, 
                                              self.euler_to_quaternion(0.0, l_elbow_pitch, 0.0)))

        # 차례대로 브로드캐스팅
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
    
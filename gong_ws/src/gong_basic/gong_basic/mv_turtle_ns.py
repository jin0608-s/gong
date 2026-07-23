import math
    
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Color, Pose


class Move_tutle(Node):
    def __init__(self):
        super().__init__("move_turtle")  # 노드 이름
        # timer 등록
        self.create_timer(0.1, self.timer_callback)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Pose, "pose", self.pose_callback, 10)
        self.create_subscription(Color, "color_sensor", self.color_callback, 10)
        self.count = 0.0
        self.pose = Pose()
        self.color = Color()

    def timer_callback(self):
        msg = Twist()  # DDS 에 보낼 객체 초기화
        
        current_step = int(self.count)
        
        # 범위를 10에서 11로 늘려서 마지막 5번째 선이 끝까지 그려지도록 만듭니다.
        if current_step < 12:  
            if current_step % 2 == 0:
                # 짝수 단계 (0, 2, 4, 6, 8, 10): 앞으로 쭉 직진
                msg.linear.x = 2.5
                msg.angular.z = 0.0
            else:
                # 홀수 단계 (1, 3, 5, 7, 9): 제자리에서 회전 (144도)
                msg.linear.x = 0.0
                msg.angular.z = 2.513
        else:
            # 완벽한 오각별을 완성하면 정지
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.pub.publish(msg)  # DDS 로 보내는 기능 수행
        self.count += 0.1


    def pose_callback(self, msg: Pose):
        self.pose = msg

    def color_callback(self, msg: Color):
        self.color = msg


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Move_tutle()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
    
    
# ros2 run turtlesim turtlesim_node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class M1sub(Node):
    def __init__(self):
        super().__init__("m1_sub") #노드 이름
        self.sub = self.create_subscription(String, "massage1", self.sub_callback, 10)

    def sub_callback(self, msg: String):
        self.get_logger().info(f'[m1 sub] Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args) #rmw 활성화
    node = M1sub()
    
    try:
        rclpy.spin(node) #블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트")
    finally:
        node.destroy_node() #노드 종료
            
        
if __name__=="__main__":
    main()


'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class M_sub(Node):
    def __init__(self):
        super().__init__("msub")  # 노드 이름
        # subscription callback 등록
        self.create_subscription(String, "message1", self.sub_callback, 10)

    def sub_callback(self, msg: String):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = M_sub()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
import message_filters


class Mtsub(Node):
    def __init__(self):
        super().__init__("mt_sub") #노드 이름
        #
        self.msg1_sub = message_filters.Subscriber(self, String, 'massage1')
        self.time_sub = message_filters.Subscriber(self, Header, 'time')
        
        # ApproximateTimeSynchronizer로 두 토픽을 0.1초 오차 범위 내에서 동기화
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.msg1_sub, self.time_sub],
            queue_size=10,
            slop=0.1
        )
        # 동기화된 데이터가 들어오면 실행할 콜백 함수를 연결합니다.
        self.ts.registerCallback(self.synchronized_callback)
        
    def synchronized_callback(self, msg1, time_msg):
        # 두 토픽의 데이터가 매칭되어 동시에 들어오는 콜백 함수
        self.get_logger().info(
            f'[mt sub] Sync Success!\n'
            f'  -> message1: {msg1.data}\n'
            f'  -> time stamp: {time_msg.stamp.sec}.{time_msg.stamp.nanosec}'
        )


def main(args=None):
    rclpy.init(args=args) #rmw 활성화
    node = Mtsub()
    
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
from std_msgs.msg import Header, String


class Mt_sub(Node):
    def __init__(self):
        super().__init__("mtsub")  # 노드 이름
        # subscription callback 등록
        self.create_subscription(String, "message1", self.sub_callback, 10)
        self.create_subscription(Header, "time", self.sub_callback2, 10)

    def sub_callback2(self, msg: Header):
        self.get_logger().info(f"{msg.stamp.sec}")

    def sub_callback(self, msg: String):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Mt_sub()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
'''
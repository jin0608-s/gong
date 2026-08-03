# 카메라를 이미지를 DDS에 publish하고 imshow로 화면에 표시
# 원을 특장 위치 10곳에 랜덤으로 이동 시키는 코드를 작성하세요.
# 원이 이동하면 라인이 그려지는 효과도 추가하세요.
# 10곳을 모두 돌면 그려진 도형을 다 지우고 처음부터 실행하게 하세요.





import random
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CircleFollow(Node):

    def __init__(self):
        super().__init__("circle_follow")

        # 30프레임 주기로 영상 생성 및 퍼블리시 콜백 등록
        self.create_timer(1 / 30, self.img_gen_callback)
        cv2.namedWindow("Camera")

        # ROS 2 퍼블리셔 선언 (DDS를 통한 토픽 전송)
        self.pub = self.create_publisher(Image, "camera/image_raw", 10)
        self.pub_info = self.create_publisher(CameraInfo, "camera/camera_info", 10)

        # CvBridge 객체 생성
        self.bridge = CvBridge()

        self.width = 640
        self.height = 480

        # GStreamer를 통한 웹캠(v4l2) 연결
        pipeline = (
            "v4l2src device=/dev/video0 ! "
            f"image/jpeg,width={self.width},height={self.height},framerate=30/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true sync=false"
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        self.camera_info = self.create_camera_info()

        # --- 요구사항 구현을 위한 변수 초기화 ---
        self.target_count = 10  # 순회할 총 위치 개수 (10곳)
        self.current_target_idx = 0  # 현재 방문 중인 인덱스
        self.points = []  # 10곳의 랜덤 좌표 리스트
        self.path_history = []  # 지나온 위치들을 저장할 리스트 (라인 그리기용)

        # 1초마다 원의 위치를 다음 랜덤 위치로 이동시키는 타이머 생성
        self.create_timer(1.0, self.update_target_position)

        # 초기 10개의 랜덤 위치 생성
        self.generate_random_points()

    def generate_random_points(self):
        """화면 영역 내에서 안전하게 이동할 수 있는 10개의 랜덤 좌표 생성"""
        self.points = []
        for _ in range(self.target_count):
            # 테두리 여백을 60픽셀씩 두어 화면 밖으로 나가지 않도록 설정
            rx = random.randint(60, self.width - 60)
            ry = random.randint(60, self.height - 60)
            self.points.append((rx, ry))

    def update_target_position(self):
        """1초마다 호출되어 원을 다음 랜덤 위치로 이동시키고 궤적을 기록"""
        if not self.points:
            return

        # 현재 위치를 궤적 기록 리스트에 추가
        current_pt = self.points[self.current_target_idx]
        self.path_history.append(current_pt)

        # 다음 인덱스로 이동
        self.current_target_idx += 1

        # 10곳을 모두 돌았을 경우: 도형(선)을 모두 지우고 처음부터 다시 실행
        if self.current_target_idx >= self.target_count:
            self.current_target_idx = 0
            self.path_history.clear()  # 그려진 라인 전체 삭제
            self.generate_random_points()  # 새로운 10곳의 위치를 다시 랜덤 생성

    def create_camera_info(self):
        msg = CameraInfo()
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = "plumb_bob"

        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = 600.0
        fy = 600.0
        cx = self.width / 2.0
        cy = self.height / 2.0

        # K matrix
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        # Rectification matrix
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # Projection matrix
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def img_gen_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # 1. 화면 테두리 사각형 그리기
        cv2.rectangle(
            frame,
            (10, 10),
            (self.width - 10, self.height - 10),
            (255, 0, 0),
            3,
        )

        # 2. 원이 이동함에 따라 지나온 자리에 라인(궤적) 그리기
        if len(self.path_history) > 1:
            for i in range(len(self.path_history) - 1):
                cv2.line(
                    frame,
                    self.path_history[i],
                    self.path_history[i + 1],
                    (0, 255, 0),  # 초록색 라인
                    2,
                    lineType=cv2.LINE_AA,
                )

        # 3. 현재 원이 위치해야 할 특정 위치에 빨간색 원 그리기
        if self.points:
            current_pt = self.points[self.current_target_idx]
            cv2.circle(frame, current_pt, 15, (0, 0, 255), -1)  # 빨간색 채워진 원
            cv2.putText(
                frame,
                f"Target: {self.current_target_idx+1}/10",
                (current_pt[0] - 35, current_pt[1] - 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )

        # 화면에 imshow 출력
        cv2.imshow("Camera", frame)
        key = cv2.waitKey(3)

        # OpenCV 이미지를 ROS 2 Image 메시지로 변환 후 헤더 타임스탬프 부여
        img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        now = self.get_clock().now().to_msg()
        img.header.stamp = now
        img.header.frame_id = "camera_link"

        self.camera_info.header.stamp = now
        self.camera_info.header.frame_id = "camera_link"

        # DDS를 통해 이미지 및 카메라 정보 토픽 퍼블리시
        self.pub.publish(img)
        self.pub_info.publish(self.camera_info)

        # 'q' 키를 누르면 종료
        if key == ord("q"):
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = CircleFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("키보드 인터럽트로 종료합니다.")
    finally:
        if node.cap.isOpened():
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# TF2 발행을 위한 모듈 임포트
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ImageArucoTfNode(Node):

    def __init__(self):
        super().__init__('image_aruco_tf_node')

        # 1. 카메라 토픽 구독자(Subscriber) 생성
        # '/camera/image_raw' 토픽을 통해 퍼블리시되는 이미지를 수신합니다.
        self.subscription = self.create_subscription(
            Image, '/gripper_camera/image_raw', self.image_callback, 10
        )

        # 2. 이미지 변환을 위한 CvBridge 초기화
        self.bridge = CvBridge()

        # 3. TF 브로드캐스터(Broadcaster) 초기화 (카메라 기준 ArUco 마커 위치 TF 발행)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 4. OpenCV ArUco 관련 설정 (DICT_4X4_50 사용 예시)
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.parameters = cv2.aruco.DetectorParameters_create()

        # 카메라 내부 파라미터 (웹캠 또는 시뮬레이터 환경에 맞춰 보정 가능)
        self.camera_matrix = np.array(
            [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # 마커 실제 물리 크기 (단위: 미터, 예: 4cm = 0.04m)
        self.marker_length = 0.04

        self.get_logger().info(
            'Image Aruco TF Subscriber Node가 성공적으로 시작되었습니다.'
        )

    def image_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV BGR 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge 변환 실패: {e}')
            return

        # 그레이스케일 변환 및 ArUco 마커 검출
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        # 마커가 검출된 경우
        if ids is not None and len(ids) > 0:
            # 마커 테두리 및 ID 시각화 화면에 그리기
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # solvePnP를 이용해 각 마커의 3차원 Pose(rvec, tvec) 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # OpenCV 내장 함수로 마커 위치에 3차원 축(Axis) 그리기
                cv2.drawFrameAxes(
                    cv_image,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    0.03,
                )

                # 화면에 마커 ID 및 3차원 좌표(x, y, z) 텍스트 표시
pos_text = f'ID:{marker_id} X:{tvec[0]:.2f} Y:{tvec[1]:.2f} Z:{tvec[2]:.2f}'                # 코너 첫 번째 점 근처에 텍스트 출력
                pt = tuple(map(int, corners[i][0][0]))
                cv2.putText(
                    cv_image,
                    pos_text,
                    (pt[0], pt[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

                # --- 5. TF(Transform) 발행 ---
                t_stamped = TransformStamped()
                t_stamped.header.stamp = self.get_clock().now().to_msg()
                # 부모 프레임 (카메라 센서 기준 좌표계)
                t_stamped.header.frame_id = 'camera_link'
                # 자식 프레임 (각 마커 고유 이름 부여)
                t_stamped.child_frame_id = f'aruco_marker_{marker_id}'

                # 위치 (Translation: tvec -> x, y, z)
                t_stamped.transform.translation.x = tvec[0]
                t_stamped.transform.translation.y = tvec[1]
                t_stamped.transform.translation.z = tvec[2]

                # 회전 벡터(rvec)를 쿼터니언(Quaternion)으로 변환
                rot_matrix, _ = cv2.Rodrigues(rvec)
                # 회전 행렬을 쿼터니언으로 바꾸기 위한 변환 행렬 구성
                quaternion = self.rotation_matrix_to_quaternion(rot_matrix)

                t_stamped.transform.rotation.x = quaternion[0]
                t_stamped.transform.rotation.y = quaternion[1]
                t_stamped.transform.rotation.z = quaternion[2]
                t_stamped.transform.rotation.w = quaternion[3]

                # TF 브로드캐스트 전송
                self.tf_broadcaster.sendTransform(t_stamped)

        # OpenCV 창을 통해 실시간 영상 및 시각화 결과 출력
        cv2.imshow('ArUco Detection & TF Publisher', cv_image)
        cv2.waitKey(1)

    def rotation_matrix_to_quaternion(self, R):
        """회전 행렬(Rotation Matrix)을 쿼터니언(x, y, z, w)으로 변환하는 헬퍼 함수"""
        q = np.empty(4, dtype=np.float64)
        trace = np.trace(R)
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            q[3] = 0.25 * s
            q[0] = (R[2, 1] - R[1, 2]) / s
            q[1] = (R[0, 2] - R[2, 0]) / s
            q[2] = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return q


def main(args=None):
    rclpy.init(args=args)
    node = ImageArucoTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.ok()


if __name__ == '__main__':
    main()
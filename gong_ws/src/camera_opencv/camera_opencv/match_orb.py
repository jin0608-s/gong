# orb_keypoints와 descritptor를 사용해서 임의의 물체를 검출한다.
# 1 keypoints -> 사진 찍어서 orb로 얻기
# 2 keypoints -> 카메라 영상을 사용
# a34번 예제, camera_pub 활용해서 AI 코드 작성



import cv2
import numpy as np
import rclpy
from rclpy.node import Node


class OrbCameraDetector(Node):

    def __init__(self):
        super().__init__("orb_camera_detector")

        # 1. 기준 물체 이미지 로드 및 ORB 특징점 사전 추출
        target_img_path = (
            "/home/boyfriend51/temp/gong/opencv_test/data/book1.jpg"
        )
        self.src1 = cv2.imread(target_img_path)

        if self.src1 is None:
            self.get_logger().error(
                f"기준 물체 이미지를 불러올 수 없습니다: {target_img_path}"
            )
            self.img1 = None
            self.kp1, self.des1 = None, None
        else:
            self.img1 = cv2.cvtColor(self.src1, cv2.COLOR_BGR2GRAY)
            self.orbF = cv2.ORB_create(nfeatures=1500)
            self.kp1, self.des1 = self.orbF.detectAndCompute(self.img1, None)
            self.get_logger().info(
                f"기준 물체 특징점 추출 완료: {len(self.kp1)}개"
            )

        # 2. BFMatcher 설정
        self.bf = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)

        # 3. GStreamer 파이프라인을 이용한 웹캠 캡처 설정
        self.width = 640
        self.height = 480
        pipeline = (
            "v4l2src device=/dev/video0 ! "
            f"image/jpeg,width={self.width},height={self.height},framerate=30/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true sync=false"
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error(
                "웹캠 장치(/dev/video0)를 열 수 없습니다. GStreamer 파이프라인을 확인하세요."
            )

        # 4. 타이머 설정 (30 FPS 주기로 프레임 읽기 및 처리)
        self.create_timer(1 / 30, self.process_frame)
        cv2.namedWindow("ORB Integrated Object Detection", cv2.WINDOW_AUTOSIZE)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("웹캠 프레임을 읽어오지 못했습니다.")
            return

        display_frame = frame.copy()

        if self.img1 is not None and self.des1 is not None:
            img2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            kp2, des2 = self.orbF.detectAndCompute(img2, None)

            if des2 is not None and len(kp2) >= 5:
                matches = self.bf.match(self.des1, des2)
                if len(matches) >= 5:
                    matches = sorted(matches, key=lambda m: m.distance)
                    minDist = matches[0].distance
                    good_matches = list(
                        filter(lambda m: m.distance < 8 * minDist, matches)
                    )

                    if len(good_matches) >= 4:
                        src1_pts = np.float32(
                            [self.kp1[m.queryIdx].pt for m in good_matches]
                        )
                        src2_pts = np.float32(
                            [kp2[m.trainIdx].pt for m in good_matches]
                        )

                        H, mask = cv2.findHomography(
                            src1_pts, src2_pts, cv2.RANSAC, 5.0
                        )

                        if H is not None:
                            mask_matches = mask.ravel().tolist()
                            h, w = self.img1.shape
                            pts = np.float32(
                                [[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]
                            ).reshape(-1, 1, 2)
                            pts2 = cv2.perspectiveTransform(pts, H)

                            display_frame = cv2.polylines(
                                frame, [np.int32(pts2)], True, (0, 255, 0), 3
                            )

                            draw_params = dict(
                                matchColor=(0, 255, 0),
                                singlePointColor=None,
                                matchesMask=mask_matches,
                                flags=2,
                            )
                            dst2 = cv2.drawMatches(
                                self.src1,
                                self.kp1,
                                display_frame,
                                kp2,
                                good_matches,
                                None,
                                **draw_params,
                            )
                            display_frame = dst2

        # 무조건 카메라 창 출력
        cv2.imshow("ORB Integrated Object Detection", display_frame)
        key = cv2.waitKey(1)
        if key == ord("q"):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = OrbCameraDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "cap") and node.cap.isOpened():
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
    
'''
 강사님 코드

from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class OrbObjectDetector(Node):
    def __init__(self):
        super().__init__("orb_object_detector")

        # -------------------------------------------------
        # 기본 설정
        # -------------------------------------------------
        self.width = 640
        self.height = 480
        self.frame_id = "camera_link"

        # ORB 및 물체 검출 설정
        self.orb_feature_count = 2500
        self.ratio_threshold = 0.85
        self.min_good_matches = 4
        self.min_inliers = 4
        self.ransac_threshold = 6.0

        # -------------------------------------------------
        # ROS publisher
        # -------------------------------------------------
        self.image_pub = self.create_publisher(
            Image,
            "camera/image_raw",
            10,
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            "camera/camera_info",
            10,
        )

        self.bridge = CvBridge()

        # -------------------------------------------------
        # ORB 특징점 검출기와 Matcher
        # -------------------------------------------------
        self.orb = cv2.ORB_create(
            nfeatures=self.orb_feature_count,
            scaleFactor=1.2,
            nlevels=10,
        )

        # ORB descriptor는 binary descriptor이므로 Hamming 거리 사용
        self.matcher = cv2.BFMatcher(
            cv2.NORM_HAMMING,
            crossCheck=False,
        )

        # -------------------------------------------------
        # 기준 이미지 face.jpg 읽기
        # -------------------------------------------------
        script_dir = Path(__file__).resolve().parent
        self.reference_path = script_dir / "card.jpg"

        self.reference_color = cv2.imread(str(self.reference_path))
        # 비율에 맞게 해상도 수정
        self.reference_color = cv2.resize(self.reference_color, (120, 250))

        if self.reference_color is None:
            raise FileNotFoundError(f"참조 이미지를 읽을 수 없습니다: {self.reference_path}")

        self.reference_gray = cv2.cvtColor(
            self.reference_color,
            cv2.COLOR_BGR2GRAY,
        )

        # 기준 이미지에서 ORB keypoint와 descriptor를 한 번만 계산
        self.reference_keypoints, self.reference_descriptors = self.orb.detectAndCompute(
            self.reference_gray,
            None,
        )

        if self.reference_descriptors is None or len(self.reference_keypoints) < 4:
            raise RuntimeError(
                "face.jpg에서 충분한 ORB 특징점을 찾을 수 없습니다. "
                "무늬와 경계가 잘 보이는 이미지를 사용하세요."
            )

        self.reference_height, self.reference_width = self.reference_gray.shape

        self.get_logger().info(f"참조 이미지: {self.reference_path}")
        self.get_logger().info(f"참조 이미지 특징점 수: {len(self.reference_keypoints)}")

        # -------------------------------------------------
        # 카메라 열기
        # -------------------------------------------------
        pipeline = (
            "v4l2src device=/dev/video0 ! "
            f"image/jpeg,width={self.width},height={self.height},"
            "framerate=30/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        self.cap = cv2.VideoCapture(
            pipeline,
            cv2.CAP_GSTREAMER,
        )

        if not self.cap.isOpened():
            self.get_logger().warning(
                "GStreamer로 카메라를 열지 못했습니다. 기본 V4L2 방식을 시도합니다."
            )

            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            raise RuntimeError("/dev/video0 카메라를 열 수 없습니다.")

        # 실제 카메라 해상도 확인
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if actual_width > 0 and actual_height > 0:
            self.width = actual_width
            self.height = actual_height

        self.camera_info = self.create_camera_info()

        # OpenCV 출력 창
        cv2.namedWindow(
            "ORB Object Detection",
            cv2.WINDOW_NORMAL,
        )

        cv2.namedWindow(
            "ORB Matches",
            cv2.WINDOW_NORMAL,
        )

        # 30 Hz 타이머
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.image_callback,
        )

        self.get_logger().info(
            "ORB 물체 검출 노드를 시작합니다. 종료하려면 영상 창에서 q를 누르세요."
        )

    def create_camera_info(self):
        """CameraInfo 메시지를 생성한다."""
        msg = CameraInfo()

        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = "plumb_bob"

        # 실제 카메라 calibration 값이 있다면 반드시 교체할 것
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = 600.0
        fy = 600.0
        cx = self.width / 2.0
        cy = self.height / 2.0

        # Camera intrinsic matrix
        msg.k = [
            fx,
            0.0,
            cx,
            0.0,
            fy,
            cy,
            0.0,
            0.0,
            1.0,
        ]

        # Rectification matrix
        msg.r = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]

        # Projection matrix
        msg.p = [
            fx,
            0.0,
            cx,
            0.0,
            0.0,
            fy,
            cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]

        return msg

    def detect_object(self, frame):
        """
        현재 카메라 프레임에서 face.jpg와 같은 물체를 찾는다.

        반환값:
            result_frame: 검출 결과가 표시된 영상
            match_image: 특징점 매칭 결과 영상
            detected: 검출 성공 여부
            good_match_count: ratio test를 통과한 매칭 개수
            inlier_count: Homography의 inlier 개수
        """
        result_frame = frame.copy()
        frame_gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY,
        )

        # 카메라 프레임의 ORB keypoint와 descriptor 추출
        frame_keypoints, frame_descriptors = self.orb.detectAndCompute(
            frame_gray,
            None,
        )

        if frame_descriptors is None or frame_keypoints is None or len(frame_keypoints) < 4:
            self.draw_status(
                result_frame,
                "No camera descriptors",
                0,
                0,
                False,
            )
            return result_frame, None, False, 0, 0

        # 각 기준 descriptor에 대해 가까운 매칭 2개 검색
        knn_matches = self.matcher.knnMatch(
            self.reference_descriptors,
            frame_descriptors,
            k=2,
        )

        # Lowe ratio test
        good_matches = []

        for match_pair in knn_matches:
            # 일부 descriptor는 매칭 결과가 2개 미만일 수 있음
            if len(match_pair) < 2:
                continue

            best_match, second_match = match_pair

            if best_match.distance < self.ratio_threshold * second_match.distance:
                good_matches.append(best_match)

        detected = False
        inlier_count = 0
        homography_mask = None

        if len(good_matches) >= self.min_good_matches:
            reference_points = np.float32(
                [self.reference_keypoints[match.queryIdx].pt for match in good_matches]
            ).reshape(-1, 1, 2)

            frame_points = np.float32(
                [frame_keypoints[match.trainIdx].pt for match in good_matches]
            ).reshape(-1, 1, 2)

            # RANSAC으로 잘못된 매칭을 제외하며 Homography 계산
            homography, homography_mask = cv2.findHomography(
                reference_points,
                frame_points,
                cv2.RANSAC,
                self.ransac_threshold,
            )

            if homography is not None and homography_mask is not None:
                inlier_count = int(homography_mask.ravel().sum())

                if inlier_count >= self.min_inliers:
                    reference_corners = np.float32(
                        [
                            [0, 0],
                            [self.reference_width - 1, 0],
                            [
                                self.reference_width - 1,
                                self.reference_height - 1,
                            ],
                            [
                                0,
                                self.reference_height - 1,
                            ],
                        ]
                    ).reshape(-1, 1, 2)

                    detected_corners = cv2.perspectiveTransform(
                        reference_corners,
                        homography,
                    )

                    if self.is_valid_polygon(
                        detected_corners,
                        frame.shape,
                    ):
                        detected = True

                        polygon = np.int32(detected_corners)

                        cv2.polylines(
                            result_frame,
                            [polygon],
                            True,
                            (0, 255, 0),
                            3,
                            cv2.LINE_AA,
                        )

                        # 검출 영역 중심점 계산
                        center = np.mean(
                            detected_corners.reshape(-1, 2),
                            axis=0,
                        ).astype(int)

                        cv2.circle(
                            result_frame,
                            tuple(center),
                            6,
                            (0, 0, 255),
                            -1,
                        )

                        cv2.putText(
                            result_frame,
                            "OBJECT DETECTED",
                            (
                                max(10, center[0] - 100),
                                max(30, center[1] - 20),
                            ),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 255, 0),
                            2,
                            cv2.LINE_AA,
                        )

        self.draw_status(
            result_frame,
            "Detected" if detected else "Not detected",
            len(good_matches),
            inlier_count,
            detected,
        )

        # 매칭 결과 영상 만들기
        match_image = self.create_match_image(
            frame,
            frame_keypoints,
            good_matches,
            homography_mask,
        )

        return (
            result_frame,
            match_image,
            detected,
            len(good_matches),
            inlier_count,
        )

    def is_valid_polygon(self, corners, frame_shape):
        """
        perspectiveTransform 결과가 지나치게 크거나 비정상적인지 검사한다.
        """
        if corners is None or corners.shape != (4, 1, 2):
            return False

        points = corners.reshape(4, 2)

        if not np.all(np.isfinite(points)):
            return False

        frame_height, frame_width = frame_shape[:2]

        polygon_area = abs(cv2.contourArea(points.astype(np.float32)))

        frame_area = frame_width * frame_height

        # 너무 작거나 화면보다 지나치게 큰 polygon 제외
        if polygon_area < 400:
            return False

        if polygon_area > frame_area * 1.5:
            return False

        # 화면 바깥으로 지나치게 벗어난 결과 제외
        margin_x = frame_width
        margin_y = frame_height

        if np.any(points[:, 0] < -margin_x):
            return False

        if np.any(points[:, 0] > frame_width + margin_x):
            return False

        if np.any(points[:, 1] < -margin_y):
            return False

        if np.any(points[:, 1] > frame_height + margin_y):
            return False

        return True

    def create_match_image(
        self,
        frame,
        frame_keypoints,
        good_matches,
        homography_mask,
    ):
        """기준 이미지와 카메라 영상의 매칭 상태를 표시한다."""
        if not good_matches:
            return None

        if homography_mask is not None:
            matches_mask = homography_mask.ravel().tolist()
        else:
            matches_mask = None

        # 화면이 복잡해지는 것을 막기 위해 최대 50개만 표시
        display_matches = good_matches[:50]

        if matches_mask is not None:
            display_mask = matches_mask[:50]
        else:
            display_mask = None

        draw_params = {
            "matchColor": (0, 255, 0),
            "singlePointColor": None,
            "flags": cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
        }

        if display_mask is not None:
            draw_params["matchesMask"] = display_mask

        return cv2.drawMatches(
            self.reference_color,
            self.reference_keypoints,
            frame,
            frame_keypoints,
            display_matches,
            None,
            **draw_params,
        )

    def draw_status(
        self,
        frame,
        status,
        good_matches,
        inliers,
        detected,
    ):
        """영상 왼쪽 위에 검출 상태를 표시한다."""
        status_color = (0, 255, 0) if detected else (0, 0, 255)

        cv2.rectangle(
            frame,
            (5, 5),
            (330, 100),
            (0, 0, 0),
            -1,
        )

        cv2.putText(
            frame,
            f"Status: {status}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            status_color,
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"Good matches: {good_matches}",
            (15, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"RANSAC inliers: {inliers}",
            (15, 87),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def image_callback(self):
        """카메라 영상을 처리하고 ROS 토픽으로 발행한다."""
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warning("카메라 프레임을 읽지 못했습니다.")
            return

        (
            result_frame,
            match_image,
            detected,
            good_match_count,
            inlier_count,
        ) = self.detect_object(frame)

        # OpenCV 화면 출력
        cv2.imshow(
            "ORB Object Detection",
            result_frame,
        )

        if match_image is not None:
            cv2.imshow(
                "ORB Matches",
                match_image,
            )

        key = cv2.waitKey(1) & 0xFF

        # ROS Image 메시지 변환 및 발행
        image_msg = self.bridge.cv2_to_imgmsg(
            result_frame,
            encoding="bgr8",
        )

        now = self.get_clock().now().to_msg()

        image_msg.header.stamp = now
        image_msg.header.frame_id = self.frame_id

        self.camera_info.header.stamp = now
        self.camera_info.header.frame_id = self.frame_id

        self.image_pub.publish(image_msg)
        self.camera_info_pub.publish(self.camera_info)

        if key == ord("q"):
            raise KeyboardInterrupt

    def destroy_node(self):
        """노드 종료 시 카메라와 OpenCV 창을 정리한다."""
        if hasattr(self, "cap"):
            self.cap.release()

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = OrbObjectDetector()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("키보드 인터럽트: 프로그램을 종료합니다.")

    except Exception as error:
        print(f"오류 발생: {error}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
'''
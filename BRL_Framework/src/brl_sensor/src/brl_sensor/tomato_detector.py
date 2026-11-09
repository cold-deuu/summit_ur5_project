#!/usr/bin/env python3
"""
source devel/setup.bash
Tomato Detector - YOLO 기반 토마토 검출 시스템

YOLO를 활용한 토마토 검출 및 Knowledge Base 업데이트
- 듀얼 카메라 시스템 (Camera1 base, Camera2 hand-eye)
- RGB + Depth → 3D 위치 계산
- TF 좌표 변환
- Single tomato rottenness scanning
- Bounding Box Freeze (CF용 KB 기반 렌더링)
- PixelTo3D 변환 (CF ADD_TOMATO용)
"""

import rospy
import rospkg
import os
import cv2
import numpy as np
from typing import Dict, List, Any
from threading import Lock

import tf
import message_filters
from scipy.spatial.transform import Rotation
from ultralytics import YOLO
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from brl_interfaces.msg import Object as ObjectMsg, Detection2D
from brl_interfaces.srv import UpdateObject, UpdateObjectRequest, TriggerSensorScan, TriggerSensorScanResponse, PixelTo3DResponse
from common_models import Robot, SceneGraph, Object

# CHANGMIM: actionlib 추가
# import actionlib
# from brl_interfaces.action import RequestFeedback


class TomatoDetector:
    """YOLO 기반 토마토 검출 + 단일 토마토 스캔"""

    def __init__(self, robot: Robot, object_update_client):
        """
        Args:
            robot: Robot 인스턴스 (중앙에서 생성된 공유 인스턴스)
            object_update_client: UpdateObject 서비스 클라이언트 (의존성 주입)
        """
        self.robot = robot
        self.object_update_client = object_update_client
        self.lock = Lock()

        # YOLO 시스템
        self.yolo_model = None
        self.cv_bridge = CvBridge()
        self.tf_listener = None
        self.rgb_intrinsic = None

        # Camera callbacks share one YOLO predictor.
        self.yolo_lock = Lock()

        # 최신 카메라 데이터 (trigger 기반 검출용)
        self.latest_rgb_msg = None
        self.latest_depth_msg = None
        self.latest_yolo_boxes = []

        # 검출 결과 (trigger 시에만 생성)
        self.latest_detection_result_cam1 = []
        self.latest_detection_result_cam2 = []

        # 단일 토마토 스캔 상태 (event-driven 방식)
        self.detection_timestamp = 0.0
        self.single_scan_active = False
        self.single_tomato_class = True  # True=ripe, False=rotten
        self.latest_scene_graph = None

        # Bounding Box Freeze 상태 (CF용)
        # frozen=True: KB 기반 렌더링 (sensor_scan 후 ~ navigation 전)
        # frozen=False: 실시간 YOLO 렌더링
        self.detection_frozen = False
        self.frozen_location = None  # freeze된 location 이름

        # 토마토 필터링 임계값 x, y, z
        self.robot_work_space = np.array([
            [0.5, 0.0, 0.6], [0.5, -1.25, 0.6],
            [-0.2, 0.0, 0.6], [-0.2, -1.25, 0.6],
            [0.5, 0.0, 1.5], [0.5, -1.25, 1.5],
            [-0.2, 0.0, 1.5], [-0.2, -1.25, 1.5]
        ])
        self.tomato_bb_wh_ratio_threshold = 100
        self.tomato_bb_area_ratio_threshold = 0.0001
        self.confidence_threshold = 0.5
        self.toamto_conf_thredhold = 0.8
        self.min_confidence = 1.0

        # YOLO 시스템 초기화
        self.init_yolo_system()
        rospy.loginfo(f"[TomatoDetector] Initialized for detection model: model/250219.pt")

        # Publisher 초기화
        self.init_visualization_publishers()
        rospy.loginfo(f"[TomatoDetector] Initialized for visualization publishers")

        # Subscriber 초기화
        self.init_camera_subscribers()

        rospy.loginfo(f"[TomatoDetector] Initialized for robot: {robot.name}")

    # ==================== YOLO System ====================

    def init_yolo_system(self):
        """YOLO 시스템 초기화"""
        try:
            rospy.loginfo("Initializing YOLO system...")

            # YOLO 모델 로드
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path("brl_sensor")
            model_path = os.path.join(pkg_path, "models/250219.pt")

            if not os.path.exists(model_path):
                rospy.logerr(f"YOLO model not found: {model_path}")
                return

            rospy.loginfo(f"Loading YOLO model from: {model_path}")
            self.yolo_model = YOLO(model_path, verbose=False)
            self.cv_bridge = CvBridge()  # ROS Image ↔ OpenCV 변환기
            rospy.loginfo("YOLO model loaded successfully")

            # TF listener 초기화 (Camera → Robot Base 좌표 변환용)
            self.tf_listener = tf.TransformListener()

            # 카메라 intrinsic 파라미터 (/camera1/color/camera_info 기준)
            self.rgb_intrinsic = {
                "fx": 606.9737548828125,
                "fy": 605.923583984375,
                "cx": 319.1101379394531,
                "cy": 251.8778533935547
            }
            rospy.loginfo("YOLO system initialization completed")


        except Exception as e:
            rospy.logerr(f"Failed to initialize YOLO system: {e}")


    # ==================== Camera Subscribers ====================

    def init_camera_subscribers(self):
        """카메라 데이터 구독 초기화"""
        try:
            # YOLO 모델 없으면 경고만 출력 (카메라 구독은 계속)
            if not self.yolo_model:
                rospy.logwarn("YOLO model not loaded, YOLO detection will be skipped")

            # 카메라 토픽
            rgb_topic_cam1 = f"/{self.robot.name}/camera1/color/image_raw"  # camera1 -> camera /brl_robot/camera/color/image_raw
            depth_topic_cam1 = f"/{self.robot.name}/camera1/aligned_depth_to_color/image_raw"  # camera1 -> camera  /brl_robot/camera/aligned_depth_to_color/image_raw
            rgb_topic_cam2 = f"/{self.robot.name}/camera2/color/image_raw" 
            rospy.loginfo(f"Subscribing to camera topics:\n- {rgb_topic_cam1}\n- {depth_topic_cam1}\n- {rgb_topic_cam2}")

            # # Camera1: RGB + Depth 동기화 (message_filters 사용)
            rgb_sub_cam1 = message_filters.Subscriber(rgb_topic_cam1, Image)
            depth_sub_cam1 = message_filters.Subscriber(depth_topic_cam1, Image)
            rospy.loginfo(f"{rgb_sub_cam1}\n{depth_sub_cam1}")

            
            ts_cam1 = message_filters.ApproximateTimeSynchronizer(
                [rgb_sub_cam1, depth_sub_cam1],
                queue_size=30,
                slop=0.3  # 100ms 허용 오차
            )
            ts_cam1.registerCallback(self.perform_yolo_cam1_callback)
            rospy.logwarn_throttle(1, "[Camera1] SYNC callback CALLED")

            # Camera2: RGB only (YOLO - Visual Servoing용)
            rospy.Subscriber(rgb_topic_cam2, Image, self.perform_yolo_cam2_callback, queue_size=1)

            # NOTE: Scene Graph는 sensor_node.py에서 변환 후 update_scene_graph()로 전달

            rospy.loginfo("Camera subscribers initialized (message_filters)")

        except Exception as e:
            rospy.logerr(f"Failed to initialize camera subscribers: {e}")

    def update_scene_graph(self, scene: SceneGraph):
        """
        Scene Graph 업데이트 (sensor_node.py에서 호출)

        Args:
            scene: Pydantic SceneGraph (RosConverter로 변환된)
        """
        self.latest_scene_graph = scene

        # Navigation 시작 감지 → freeze 해제
        # (다음 location으로 이동 시 다시 실시간 YOLO 모드로)
        try:
            robot = scene.get_robot_by_name(self.robot.name)
            if robot and robot.status == "navigating":
                if self.detection_frozen:
                    rospy.loginfo("[TomatoDetector] Navigation detected, UNFREEZING boxes")
                    self.detection_frozen = False
                    self.frozen_location = None
        except Exception as e:
            rospy.logdebug(f"[TomatoDetector] Error checking navigation: {e}")

    # ==================== YOLO Detection ====================
    def perform_yolo_cam1_callback(self, rgb_msg: Image, depth_msg: Image):
        """Camera1 YOLO 검출 (실시간 시각화 + Box 저장)"""
        try:
            # None 체크
            if rgb_msg is None or depth_msg is None:
                rospy.logwarn_throttle(5, "[Camera1] Waiting for camera data...")
                return

            # 이미지 저장 (trigger 시 사용)
            self.latest_rgb_msg = rgb_msg
            self.latest_depth_msg = depth_msg

            # YOLO 시스템 체크
            if self.latest_rgb_msg is None:
                rospy.logwarn_throttle(10, "[Camera1] RGB image not available, skipping detection")
                return
            
            if not self.yolo_model:
                rospy.logwarn_throttle(10, "[Camera1] YOLO model not loaded, skipping detection")
                return

            if not self.cv_bridge:
                rospy.logwarn_throttle(10, "[Camera1] CvBridge not initialized, skipping detection")
                return

            if not self.rgb_intrinsic:
                rospy.logwarn_throttle(10, "[Camera1] Camera intrinsics not set, skipping detection")
                return

            # 이미지 변환
            try:
                cv_image_rgb = self.cv_bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
                cv_image_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)
            except Exception as e:
                rospy.logerr(f"[Camera1] Failed to convert images: {e}")
                return

            # YOLO 검출 수행
            with self.yolo_lock:
                detection_result = self.yolo_model.predict(cv_image_rgb, verbose=False)[0]

            detected_count = 0
            good_bounding_boxes = []
            image_shape = cv_image_rgb.shape[:2]  # (height, width)

            # 필터링만 수행 (Object 생성은 trigger 시!)
            rospy.loginfo_throttle(5, "[Camera1] The number of YOLO detections: {}".format(len(detection_result.boxes)))
            for idx, box in enumerate(detection_result.boxes):
                try:
                    # Bounding box 정보 추출: [x1, y1, x2, y2]
                    xyxy = box.xyxy[0].tolist()

                    # === Depth 계산 ===
                    # 토마토는 구형이므로 bounding box 전체 영역의 depth를 사용합니다.
                    # - 중앙 픽셀만 사용하면: 토마토 표면의 가장 가까운 점만 측정
                    # - 전체 영역 평균 사용: 토마토 중심까지의 거리에 더 가까운 값
                    # - 또한 depth 센서 노이즈를 평균화하여 더 안정적인 값 획득

                    # Bounding box 중심점 (카메라 좌표계 변환에 사용)
                    center_u = (xyxy[0] + xyxy[2]) / 2
                    center_v = (xyxy[1] + xyxy[3]) / 2

                    # Bounding box 전체 영역에서 depth 추출
                    u_range = [int(xyxy[0]), int(xyxy[2])]
                    v_range = [int(xyxy[1]), int(xyxy[3])]
                    depth_slice = cv_image_depth[v_range[0]:v_range[1]+1, u_range[0]:u_range[1]+1].flatten()

                    # 유효 depth 필터링: 100mm ~ 10000mm (너무 가깝거나 먼 값 제외)
                    valid_depths = depth_slice[(depth_slice > 100) & (depth_slice < 10000)]
                    if len(valid_depths) == 0:
                        print("[DEBUG][Camera1] No valid depth pixels for detection {}, skipping".format(idx))
                        continue

                    # 평균(mean) 사용: 전체 영역의 대표값으로 노이즈 감소
                    # (cf. pixel_to_3d에서는 3x3 윈도우 + median 사용)
                    depth_mean = np.mean(valid_depths) * 0.001  # mm → m

                    # === 카메라 좌표계 → 3D 좌표 변환 ===
                    # Pinhole 카메라 모델: x = (u - cx) * z / fx, y = (v - cy) * z / fy
                    x_cam = (center_u - self.rgb_intrinsic["cx"]) * depth_mean / self.rgb_intrinsic["fx"]
                    y_cam = (center_v - self.rgb_intrinsic["cy"]) * depth_mean / self.rgb_intrinsic["fy"]
                    z_cam = depth_mean

                    # 로봇 base 좌표계로 변환 (workspace 필터링용)
                    object_pos_in_robot_frame = self.transform_camera_to_robot_base_cam1([x_cam, y_cam, z_cam])

                    # TF 실패 시 스킵 ({0,0,0} 반환됨)
                    if object_pos_in_robot_frame["x"] == 0.0 and object_pos_in_robot_frame["y"] == 0.0:
                        rospy.logwarn_throttle(5, f"[Camera1] TF failed for detection {idx}, skipping")
                        print("[DEBUG][Camera1] TF failed for detection {}, skipping".format(idx))
                        continue

                    # 토마토 필터링 (4단계: confidence, ratio, workspace, min_size)
                    if self.filter_tomatoes(box=box, robot_position=object_pos_in_robot_frame, image_shape=image_shape):
                        good_bounding_boxes.append(box)
                        detected_count += 1
                        rospy.loginfo_throttle(10, f"[Camera1] Detection {idx} accepted after filtering")

                except Exception as e:
                    rospy.logerr(f"Error processing detection {idx}: {e}")

            # During a held-tomato scan, display and classify the same ROI box [0].
            scan_active = self.single_scan_active
            if scan_active:
                good_bounding_boxes = self._select_single_scan_boxes(good_bounding_boxes, image_shape)

            # Box만 저장 (Object 생성은 trigger 시!)
            self.latest_yolo_boxes = good_bounding_boxes
            # Annotated 이미지 생성: frozen 여부에 따라 소스 선택
            # - frozen=True: KB 기반 (CF 수정 반영)
            # - frozen=False: 실시간 YOLO
            if self.detection_frozen and not scan_active:
                kb_boxes = self._get_boxes_from_kb()
                new_yolo_image = self.draw_bounding_boxes_from_kb(cv_image_rgb, kb_boxes)
                rospy.logdebug_throttle(5, f"[Camera1] Drawing {len(kb_boxes)} KB boxes (frozen)")
            else:
                new_yolo_image = self.draw_bounding_boxes(cv_image_rgb, good_bounding_boxes)

            frame_id = f"{self.robot.name}_camera1_color_optical_frame"

            # Raw 이미지 발행 (WebRTC, RViz용)
            self._publish_raw_image(new_yolo_image, self.camera1_annotated_pub_raw, frame_id)

            # 카메라 timestamp 발행 (BT 이미지 안정화용)
            self.camera_timestamp_pub.publish(rgb_msg.header)

            # Single Tomato Scan 중이면 즉시 rotten 체크 (event-driven)
            # Changmin add: rotten 체크 로직 수정, min confidence 추가
            if scan_active:
                if not good_bounding_boxes:
                    rospy.logwarn_throttle(
                        2,
                        "[SingleScan] No tomato center inside scan ROI; "
                        "waiting for the next frame",
                    )
                    return

                box = good_bounding_boxes[0]
                detection_class = int(box.cls[0].item())
                detection_conf = float(box.conf[0].item())
                self.min_confidence = min(self.min_confidence, detection_conf)

                if detection_class == 2:  # rotten
                    with self.lock:
                        self.single_tomato_class = False
                    rospy.loginfo("[SingleScan] Rotten detected in real-time!")
                    # break

        except Exception as e:
            rospy.logerr(f"Error in YOLO detection (camera1): {e}")

    def perform_yolo_cam2_callback(self, rgb_msg: Image):
        """Camera2 YOLO: publish live video and a target for visual servoing."""
        try:
            if rgb_msg is None:
                return
            if self.cv_bridge is None:
                rospy.logwarn_throttle(10, "[Camera2] CvBridge not initialized")
                return

            image = self.cv_bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            good_boxes = []
            if self.yolo_model is not None:
                with self.yolo_lock:
                    result = self.yolo_model.predict(image, verbose=False)[0]
                for box in result.boxes:
                    if self.filter_tomatoes(box, robot_position=None, image_shape=image.shape[:2]):
                        good_boxes.append(box)
                rospy.loginfo_throttle(5, f"[Camera2] YOLO targets: {len(good_boxes)}")
            else:
                rospy.logwarn_throttle(10, "[Camera2] YOLO model not loaded; showing raw video")

            if self.single_scan_active:
                good_boxes = good_boxes[:1]
            annotated = self.draw_bounding_boxes(image, good_boxes)
            frame_id = f"{self.robot.name}_camera2_color_optical_frame"
            self._publish_raw_image(annotated, self.camera2_annotated_pub_raw, frame_id)
            self._publish_best_target(good_boxes, rgb_msg.header)
        except Exception as e:
            self._publish_empty_best_target(rgb_msg.header)
            rospy.logerr(f"Error in YOLO detection (camera2): {e}")

    # ==================== Filtering ====================

    @staticmethod
    def _select_single_scan_boxes(boxes, image_shape):
        """Keep only ROI match [0], preserving the existing held-tomato scan ROI."""
        height, width = image_shape
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
            if width * 0.30 <= center_x <= width * 0.70 and center_y <= height * 0.50:
                return [box]
        return []

    def filter_tomatoes(self, box, robot_position=None, image_shape=None):
        """
        토마토 필터링 (4단계)

        Args:
            box: YOLO detection box
            robot_position: Object position in robot base frame (dict with x, y, z)
            image_shape: (height, width) of the image for dynamic min_area calculation

        Returns:
            bool: True if valid tomato detection
        """
        xyxy = box.xyxy[0].tolist()
        width = abs(xyxy[0] - xyxy[2])
        height = abs(xyxy[1] - xyxy[3])
        confidence = float(box.conf[0])

        # 기준 0: Confidence 임계값
        cri_0 = confidence >= self.confidence_threshold

        # 기준 1: Aspect ratio (원형에 가까운지)
        bb_ratio = max(width / height, height / width)
        cri_1 = bb_ratio <= self.tomato_bb_wh_ratio_threshold

        # 기준 2: Workspace (로봇 작업 범위 내, Camera1만 적용)
        if robot_position:
            eps = 1e-6
            p = np.asarray([robot_position['x'], robot_position['y'], robot_position['z']], dtype=float)
            corners = np.asarray(self.robot_work_space, dtype=float)
            lo = corners.min(axis=0) - eps
            hi = corners.max(axis=0) + eps
            cri_2 = np.all((p >= lo) & (p <= hi))
            # cri_2 = True # 임시로 workspace 필터링 해제 TODO workspace 다시 계산
        else:
            cri_2 = True

        # 기준 3: 최소 크기 (너무 작은 검출 제거)
        if image_shape:
            img_height, img_width = image_shape
            min_area = int(img_width * img_height * self.tomato_bb_area_ratio_threshold)
        else:
            min_area = int(640 * 480 * self.tomato_bb_area_ratio_threshold)  # fallback
        cri_3 = width * height >= min_area

        # rospy.loginfo_throttle(10, f"[Filter] \n\tConf: {confidence:.2f} (cri_0: {cri_0}), "
        #                          f"\n\tRatio: {bb_ratio:.2f} (cri_1: {cri_1}), "
        #                          f"\n\tWorkspace: {robot_position} (cri_2: {cri_2}), "
        #                          f"\n\tArea: {width*height} (cri_3: {cri_3})")
        # rospy.loginfo_throttle(10, f"[Filter] \n\tIs Filtered Tomato? {cri_0 and cri_1 and cri_2 and cri_3}")
        
        return cri_0 and cri_1 and cri_2 and cri_3

    # ==================== KB-based Rendering (Freeze) ====================

    def _get_boxes_from_kb(self) -> List[dict]:
        """
        KB의 Object에서 bounding_box 정보 추출 (frozen 상태에서 사용)

        Returns:
            List of dict: [{"xyxy": [x1,y1,x2,y2], "class": int, "confidence": float, "name": str}, ...]
        """
        if not self.latest_scene_graph or not self.frozen_location:
            return []

        boxes = []

        try:
            objects = self.latest_scene_graph.get_objects_at_location(self.frozen_location)

            for obj in objects:
                if obj.type != "tomato":
                    continue
                # 삭제된 객체는 scene_graph에서 이미 제거됨 (별도 체크 불필요)

                # bounding_box property 파싱: "[x1,y1,x2,y2]"
                # Python Object.properties는 Dict[str, Any]
                bbox_str = obj.properties.get("bounding_box")
                if not bbox_str:
                    continue

                try:
                    # "[280.5,120.3,350.2,200.1]" → [280.5, 120.3, 350.2, 200.1]
                    coords = str(bbox_str).strip("[]").split(",")
                    xyxy = [float(c) for c in coords]
                    if len(xyxy) != 4:
                        continue
                except (ValueError, AttributeError):
                    continue

                tomato_class = int(obj.properties.get("tomato_class", 0))
                confidence = float(obj.properties.get("confidence", 0.0))

                boxes.append({
                    "xyxy": xyxy,
                    "class": tomato_class,
                    "confidence": confidence,
                    "name": obj.name
                })

        except Exception as e:
            rospy.logerr(f"[_get_boxes_from_kb] Error: {e}")

        return boxes

    def draw_bounding_boxes_from_kb(self, image, kb_boxes: List[dict]):
        """
        KB 기반 bounding box 그리기 (frozen 상태에서 사용)

        Args:
            image: OpenCV 이미지
            kb_boxes: _get_boxes_from_kb()에서 반환된 리스트

        Returns:
            Annotated 이미지
        """
        try:
            anno_image = image.copy()
            class_names = {0: "ripe", 1: "unripe", 2: "rotten"}
            class_colors = {0: (0, 255, 0), 1: (0, 255, 255), 2: (0, 0, 255)}

            for box_info in kb_boxes:
                xyxy = box_info["xyxy"]
                detection_class = box_info["class"]
                confidence = box_info["confidence"]

                # Box 그리기
                color = class_colors.get(detection_class, (255, 255, 255))
                cv2.rectangle(anno_image,
                            (int(xyxy[0]), int(xyxy[1])),
                            (int(xyxy[2]), int(xyxy[3])),
                            color, 2)

                # Label 그리기
                class_name = class_names.get(detection_class, "unknown")
                label = f"{class_name} {confidence:.2f}"
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                cv2.rectangle(anno_image,
                            (int(xyxy[0]), int(xyxy[1]) - label_size[1] - 10),
                            (int(xyxy[0]) + label_size[0], int(xyxy[1])),
                            color, -1)
                cv2.putText(anno_image, label,
                          (int(xyxy[0]), int(xyxy[1]) - 5),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            return anno_image

        except Exception as e:
            rospy.logerr(f"Error drawing KB boxes: {e}")
            return image

    # ==================== Visualization ====================

    def draw_bounding_boxes(self, image, bounding_boxes):
        """바운딩 박스 그리기"""
        try:
            anno_image = image.copy()
            class_names = {0: "ripe", 1: "unripe", 2: "rotten"}
            class_colors = {0: (0, 255, 0), 1: (0, 255, 255), 2: (0, 0, 255)}

            for idx, box in enumerate(bounding_boxes):
                xyxy = box.xyxy[0].tolist()
                confidence = float(box.conf[0])
                detection_class = int(box.cls[0])

                # Box 그리기
                color = class_colors.get(detection_class, (255, 255, 255))
                cv2.rectangle(anno_image,
                            (int(xyxy[0]), int(xyxy[1])),
                            (int(xyxy[2]), int(xyxy[3])),
                            color, 2)

                # Label 그리기
                class_name = class_names.get(detection_class, "unknown")
                label = f"{class_name} {confidence:.2f} (cls:{detection_class})"
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                cv2.rectangle(anno_image,
                            (int(xyxy[0]), int(xyxy[1]) - label_size[1] - 10),
                            (int(xyxy[0]) + label_size[0], int(xyxy[1])),
                            color, -1)
                cv2.putText(anno_image, label,
                          (int(xyxy[0]), int(xyxy[1]) - 5),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            return anno_image

        except Exception as e:
            rospy.logerr(f"Error creating annotated image: {e}")
            return image


    def init_visualization_publishers(self):
        """이미지 발행 Publisher 초기화"""
        try:
            # 카메라 timestamp Publisher (BT 이미지 안정화용)
            self.camera_timestamp_pub = rospy.Publisher(
                f"/{self.robot.name}/camera_timestamp",
                Header, queue_size=1
            )
            # Compressed 이미지 Publisher
            self.camera1_annotated_pub = rospy.Publisher(
                f"/{self.robot.name}/yolo_detection/camera1_image_annotated",
                CompressedImage, queue_size=1
            )
            self.camera2_annotated_pub = rospy.Publisher(
                f"/{self.robot.name}/yolo_detection/camera2_image_annotated",
                CompressedImage, queue_size=1
            )

            # Raw 이미지 Publisher (Rviz 호환용)
            self.camera1_annotated_pub_raw = rospy.Publisher(
                f"/{self.robot.name}/yolo_detection/camera1_image_annotated_raw",
                Image, queue_size=1
            )

            # Camera2: YOLO (Raw 이미지) - 기존 토픽명 유지 (WebRTC 호환)
            self.camera2_annotated_pub_raw = rospy.Publisher(
                f"/{self.robot.name}/yolo_detection/camera2_image_annotated_raw",
                Image, queue_size=1
            )

            # Visual Servoing용 best target Publisher (Camera2 YOLO)
            # 기존 토픽명 유지 (motion_planning 호환)
            self.camera2_best_target_pub = rospy.Publisher(
                f"/{self.robot.name}/yolo_detection/camera2_best_target",
                Detection2D, queue_size=1
            )
            # rospy.loginfo(f"Visualization publishers initialized (including /{self.robot.name}/camera_timestamp, camera2_best_target)")

        except Exception as e:
            rospy.logerr(f"Failed to initialize visualization publishers: {e}")

    def _publish_compressed_image(self, image_np, publisher, frame_id):
        """압축 이미지 발행"""
        try:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id
            msg.format = "jpeg"

            success, encoded_image = cv2.imencode('.jpg', image_np)
            if not success:
                rospy.logwarn(f"Failed to encode image")
                return

            # rospy.loginfo(f"Visualization publishers initialized: camera1(YOLO), camera2(YOLO), best_target")

        except Exception as e:
            rospy.logerr(f"Failed to initialize visualization publishers: {e}")

    def _publish_raw_image(self, image_np, publisher, frame_id):
        """일반 Image 메시지 발행 (Rviz 호환성)"""
        try:
            msg = self.cv_bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id
            publisher.publish(msg)

        except Exception as e:
            rospy.logerr(f"Error publishing raw image: {e}")

    def _publish_best_target(self, bounding_boxes, header):
        """
        Visual Servoing용 best target 발행

        선택 기준: bbox 면적이 가장 큰 것 (가장 가까운 토마토)

        Args:
            bounding_boxes: 필터링된 YOLO box 리스트
            header: 원본 이미지의 header (timestamp 동기화)
        """
        try:
            msg = Detection2D()
            msg.header = header

            if not bounding_boxes:
                # 검출 없음
                msg.valid = False
                msg.center_x = 0.0
                msg.center_y = 0.0
                msg.width = 0.0
                msg.height = 0.0
            else:
                # 면적이 가장 큰 bbox 선택
                best_box = None
                best_area = 0.0

                for box in bounding_boxes:
                    xyxy = box.xyxy[0].tolist()
                    width = xyxy[2] - xyxy[0]
                    height = xyxy[3] - xyxy[1]
                    area = width * height

                    if area > best_area:
                        best_area = area
                        best_box = box

                if best_box is not None:
                    xyxy = best_box.xyxy[0].tolist()
                    msg.valid = True
                    msg.center_x = (xyxy[0] + xyxy[2]) / 2.0
                    msg.center_y = (xyxy[1] + xyxy[3]) / 2.0
                    msg.width = xyxy[2] - xyxy[0]
                    msg.height = xyxy[3] - xyxy[1]
                else:
                    msg.valid = False

            self.camera2_best_target_pub.publish(msg)

        except Exception as e:
            rospy.logerr(f"Error publishing best target: {e}")


    def _publish_empty_best_target(self, header):
        """검출 없을 때 invalid target 발행"""
        try:
            msg = Detection2D()
            msg.header = header
            msg.valid = False
            msg.center_x = 0.0
            msg.center_y = 0.0
            msg.width = 0.0
            msg.height = 0.0
            self.camera2_best_target_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"Error publishing empty best target: {e}")

    # ==================== Coordinate Transformation ====================
    def transform_camera_to_robot_base_cam1(self, camera_position: List[float]) -> Dict[str, float]:
        """
        Camera1 좌표계 → Robot base 좌표계 변환

        Args:
            camera_position: 카메라 좌표계 위치 [x, y, z]

        Returns:
            Robot base 좌표계 위치 {"x": ..., "y": ..., "z": ...}
        """
        try:
            parent_frame = "robot_base_footprint"
            child_frame = "camera1_color_optical_frame_copy"
            # child_frame = "camera1_color_optical_frame"


            self.tf_listener.waitForTransform(
                parent_frame, child_frame,
                rospy.Time(0), rospy.Duration(1.0)
            )
            trans, quat = self.tf_listener.lookupTransform(
                parent_frame, child_frame, rospy.Time(0)
            )

            rot_mat = Rotation.from_quat(quat).as_matrix()
            tf_mat = np.eye(4)
            tf_mat[:3, :3] = rot_mat
            tf_mat[:3, 3] = trans

            camera_pos_homo = list(camera_position) + [1]
            object_pos_in_robot_frame = np.dot(tf_mat, camera_pos_homo)[:3]

            return {
                "x": float(object_pos_in_robot_frame[0]),
                "y": float(object_pos_in_robot_frame[1]),
                "z": float(object_pos_in_robot_frame[2])
            }

        except Exception as e:
            rospy.logerr(f"Failed to transform coordinates (cam1): {e}")
            return {"x": 0.0, "y": 0.0, "z": 0.0}

    def transform_camera_to_robot_base_cam2(self, camera_position: List[float]) -> Dict[str, float]:
        """
        Camera2 좌표계 → Robot base 좌표계 변환

        Args:
            camera_position: 카메라 좌표계 위치 [x, y, z]

        Returns:
            Robot base 좌표계 위치 {"x": ..., "y": ..., "z": ...}
        """
        try:
            parent_frame = "robot_base_footprint"
            child_frame = "camera2_color_optical_frame"

            self.tf_listener.waitForTransform(
                parent_frame, child_frame,
                rospy.Time(0), rospy.Duration(1.0)
            )
            trans, quat = self.tf_listener.lookupTransform(
                parent_frame, child_frame, rospy.Time(0)
            )

            rot_mat = Rotation.from_quat(quat).as_matrix()
            tf_mat = np.eye(4)
            tf_mat[:3, :3] = rot_mat
            tf_mat[:3, 3] = trans

            camera_pos_homo = list(camera_position) + [1]
            object_pos_in_robot_frame = np.dot(tf_mat, camera_pos_homo)[:3]

            return {
                "x": float(object_pos_in_robot_frame[0]),
                "y": float(object_pos_in_robot_frame[1]),
                "z": float(object_pos_in_robot_frame[2])
            }

        except Exception as e:
            rospy.logerr(f"Failed to transform coordinates (cam2): {e}")
            return {"x": 0.0, "y": 0.0, "z": 0.0}

    # ==================== Sensor Service Callbacks ====================
    # Note: 서비스는 sensor_node.py에서 제공하지만, 콜백 로직은 여기에 유지

    # Position 기반 중복 체크 마진 (미터)
    POSITION_MATCH_MARGIN = 0.03  # 3cm

    def _find_existing_tomato(self, new_pos: Dict[str, float], existing_objects: List[Object]) -> Object:
        """
        position 기반으로 기존 토마토 찾기

        Args:
            new_pos: 새 토마토 위치 {"x", "y", "z"}
            existing_objects: Pydantic Object 리스트

        Returns:
            매칭되는 기존 Object 또는 None
        """
        for obj in existing_objects:
            if obj.type != "tomato":
                continue

            # Pydantic Object의 position은 Position 타입
            dx = abs(new_pos["x"] - obj.position.x)
            dy = abs(new_pos["y"] - obj.position.y)
            dz = abs(new_pos["z"] - obj.position.z)

            if dx < self.POSITION_MATCH_MARGIN and dy < self.POSITION_MATCH_MARGIN and dz < self.POSITION_MATCH_MARGIN:
                rospy.logdebug(f"Found existing tomato '{obj.name}' at distance ({dx:.3f}, {dy:.3f}, {dz:.3f})")
                return obj

        return None

    def _get_existing_tomatoes_from_scene(self, target_location: str) -> List[Object]:
        """
        scene_graph에서 해당 위치의 기존 토마토들 가져오기

        Args:
            target_location: 위치 이름 (e.g., "Stem_1")

        Returns:
            해당 위치의 토마토 Object 리스트 (Pydantic)
        """
        if self.latest_scene_graph is None:
            return []

        # SceneGraph 헬퍼 메서드 사용
        objects = self.latest_scene_graph.get_objects_at_location(target_location)
        return [obj for obj in objects if obj.type == "tomato"]

    def sensor_scan_callback(self, req):
        """
        navigate이후 trigger되는 전체 토마토 디텍션 콜백 함수
        센서 스캔 트리거 콜백 - BT에서 트리거 시 Object 생성/업데이트

        중복 처리 로직:
        - 기존 토마토와 position이 POSITION_MATCH_MARGIN 내이면 → 기존 객체 업데이트
        - 그렇지 않으면 → 새 객체 생성
        """
        try:
            target_location = req.target_location
            target_object = req.target_object

            rospy.loginfo(f"[{self.robot.name}] Sensor scan triggered: location={target_location}, object={target_object}")

            if not self.yolo_model:
                rospy.logerr(f"[{self.robot.name}] YOLO system not initialized")
                return TriggerSensorScanResponse(success=False, message="YOLO not initialized")

            if self.latest_rgb_msg is None or self.latest_depth_msg is None:
                rospy.logwarn(f"[{self.robot.name}] No camera data available")
                return TriggerSensorScanResponse(success=True, message="No camera data")

            if len(self.latest_yolo_boxes) == 0:
                rospy.loginfo(f"[{self.robot.name}] No tomatoes detected")
                return TriggerSensorScanResponse(success=True, message="No detections")

            # 기존 토마토들 가져오기 (중복 체크용)
            existing_tomatoes = self._get_existing_tomatoes_from_scene(target_location)
            rospy.loginfo(f"[{self.robot.name}] Found {len(existing_tomatoes)} existing tomatoes at '{target_location}'")

            rospy.loginfo(f"[{self.robot.name}] Processing {len(self.latest_yolo_boxes)} detections for KB update")

            cv_image_depth = self.cv_bridge.imgmsg_to_cv2(self.latest_depth_msg)
            current_time = rospy.Time.now()
            objects_to_update = []
            new_count = 0
            update_count = 0

            for idx, box in enumerate(self.latest_yolo_boxes):
                try:
                    # Box 정보 추출
                    xyxy = box.xyxy[0].tolist()
                    confidence = float(box.conf[0])
                    detection_class = int(box.cls[0])

                    # 중심점 계산
                    center_u = (xyxy[0] + xyxy[2]) / 2
                    center_v = (xyxy[1] + xyxy[3]) / 2

                    # Depth 값 추출
                    u_range = [int(xyxy[0]), int(xyxy[2])]
                    v_range = [int(xyxy[1]), int(xyxy[3])]
                    depth_slice = cv_image_depth[v_range[0]:v_range[1]+1, u_range[0]:u_range[1]+1].flatten()

                    valid_depths = depth_slice[(depth_slice > 100) & (depth_slice < 10000)]
                    if len(valid_depths) == 0:
                        continue

                    depth_mean = np.mean(valid_depths) * 0.001  # mm → m

                    # 카메라 좌표계 계산
                    x_cam = (center_u - self.rgb_intrinsic["cx"]) * depth_mean / self.rgb_intrinsic["fx"]
                    y_cam = (center_v - self.rgb_intrinsic["cy"]) * depth_mean / self.rgb_intrinsic["fy"]
                    z_cam = depth_mean

                    # Robot base 좌표계로 변환
                    object_pos_in_robot_frame = self.transform_camera_to_robot_base_cam1([x_cam, y_cam, z_cam])

                    # 기존 토마토 찾기 (position 기반 중복 체크)
                    existing_tomato = self._find_existing_tomato(object_pos_in_robot_frame, existing_tomatoes)

                    # Object 메시지 생성
                    object_msg = ObjectMsg()

                    if existing_tomato:
                        # 기존 객체 업데이트 - 이름 유지, properties 업데이트
                        object_msg.name = existing_tomato.name
                        object_msg.status = existing_tomato.status  # 기존 status 유지 (picked 등)
                        update_count += 1
                        rospy.logdebug(f"Updating existing tomato: {existing_tomato.name}")
                    else:
                        # 새 객체 생성
                        object_msg.name = f"tomato_{self.robot.name}_{current_time.secs}_{idx}"
                        object_msg.status = "detected"
                        new_count += 1
                        rospy.logdebug(f"Creating new tomato: {object_msg.name}")

                    object_msg.type = "tomato"
                    object_msg.current_location_name = target_location
                    object_msg.held_by_robot_name = ""
                    object_msg.position = Point(
                        x=object_pos_in_robot_frame["x"],
                        y=object_pos_in_robot_frame["y"],
                        z=object_pos_in_robot_frame["z"]
                    )
                    object_msg.properties = [
                        f"tomato_class:{detection_class}",  # 0=ripe, 1=unripe, 2=rotten
                        f"confidence:{confidence:.3f}",
                        f"bounding_box:[{xyxy[0]:.1f},{xyxy[1]:.1f},{xyxy[2]:.1f},{xyxy[3]:.1f}]",
                        f"pixel_center:{center_u:.1f},{center_v:.1f}",
                    ]
                    object_msg.index = ""

                    objects_to_update.append(object_msg)

                except Exception as e:
                    rospy.logerr(f"Error creating object {idx}: {e}")

            # 생성된 Object 저장 (single tomato scan용)
            self.latest_detection_result_cam1 = objects_to_update

            # KB 업데이트 Changmin
            req_obj = UpdateObjectRequest()
            success_count = 0
            for index, obj in enumerate(objects_to_update):
                obj.index = f"{index+1}/{len(objects_to_update)}"  # BT에서 객체 구분용 (optional)
                req_obj.object_data = obj
                response = self.object_update_client(req_obj)
                if response.success:
                    success_count += 1
                else:
                    rospy.logwarn(f"Failed to update object: {response.message}")

            rospy.loginfo(f"[{self.robot.name}] KB update complete: {new_count} new, {update_count} updated, {success_count}/{len(objects_to_update)} success")

            # Freeze 활성화: 이후 렌더링은 KB 기반
            self.detection_frozen = True
            self.frozen_location = target_location
            rospy.loginfo(f"[{self.robot.name}] Bounding boxes FROZEN for location: {target_location}")

            return TriggerSensorScanResponse(success=True, message=f"New: {new_count}, Updated: {update_count}")

        except Exception as e:
            rospy.logerr(f"Error in sensor scan callback: {e}")
            return TriggerSensorScanResponse(success=False, message=str(e))

    # ==================== Single Tomato Scan ====================

    def scan_single_tomato(self, req, mode: str):
        """
        단일 토마토 스캔 (rottenness 판정)
        Chanmin: min confidence를 추가하여 rotten 판정을 KB에 업데이트

        역할 분리:
        - sensor: tomato_class 결정 및 KB 업데이트 (이 함수)
        - BT: status="scanned" KB 업데이트 (CheckScanedCondition)

        Args:
            req: TriggerSensorScanRequest (target_object 포함)
            mode: "start" 또는 "end"
        """
        if mode == "start":
            with self.lock:
                if self.single_scan_active:
                    rospy.logwarn("[SingleScan] Already running")
                    return TriggerSensorScanResponse(success=True)

                self.single_scan_active = True
                self.single_tomato_class = True  # 초기값: ripe
                self.min_confidence = 1.0  # 초기값: 최대값 Changmin

                # Freeze 해제: ScanTomato 중에는 실시간 YOLO 필요
                # (gripper 토마토의 rotten 여부를 실시간으로 확인)
                if self.detection_frozen:
                    rospy.loginfo("[SingleScan] UNFREEZING for real-time scan")
                    self.detection_frozen = False

                # Event-driven 방식: perform_yolo_cam1_callback에서 즉시 처리
                rospy.loginfo("[SingleScan] Started rottenness detection (event-driven)")
                return TriggerSensorScanResponse(success=True)

        elif mode == "end":
            with self.lock:
                if not self.single_scan_active:
                    rospy.logwarn("[SingleScan] Not running")
                    return TriggerSensorScanResponse(success=True)

                # tomato_class 결정
                tomato_class = "0" if self.single_tomato_class else "2"
                rospy.loginfo(f"[SingleScan] Result: {'Ripe' if self.single_tomato_class else 'Rotten'} (class={tomato_class})")

                # KB에 tomato_class 업데이트
                tomato_name = getattr(req, 'target_object', '')
                if tomato_name and self.latest_scene_graph:
                    target = self.latest_scene_graph.get_object_by_name(tomato_name)
                    if target:
                        updated = ObjectMsg()
                        updated.name = target.name
                        updated.type = target.type
                        updated.status = target.status  # BT가 "scanned"로 업데이트
                        updated.position = Point(x=target.position.x, y=target.position.y, z=target.position.z)
                        updated.current_location_name = ""
                        updated.held_by_robot_name = self.robot.name
                        # updated.properties = [f"tomato_class:{tomato_class}"] # Original
                        updated.properties = [f"tomato_class:{tomato_class}", f"confidence:{self.min_confidence:.3f}"] # Changmin

                        req_obj = UpdateObjectRequest()
                        req_obj.object_data = updated
                        try:
                            response = self.object_update_client(req_obj)
                            
                            if response.success:
                                rospy.loginfo(f"[SingleScan] KB updated: {tomato_name} tomato_class={tomato_class} confidence={self.min_confidence:.3f}")
                            else:
                                rospy.logwarn(f"[SingleScan] Failed to update KB: {response.message}")
                        except Exception as e:
                            rospy.logerr(f"[SingleScan] KB update error: {e}")
                    else:
                        rospy.logwarn(f"[SingleScan] Tomato '{tomato_name}' not found in SceneGraph")
                else:
                    rospy.logwarn("[SingleScan] No tomato_name or scene_graph, skipping KB update")

                # 상태 리셋
                self.single_scan_active = False
                self.single_tomato_class = True
                self.min_confidence = 1.0 # Changmin

                # Freeze 재활성화: ScanTomato 완료 후 다시 KB 기반 렌더링
                if self.frozen_location:
                    self.detection_frozen = True
                    rospy.loginfo(f"[SingleScan] RE-FREEZING for location: {self.frozen_location}")

                return TriggerSensorScanResponse(success=True)

        else:
            rospy.logerr(f"[SingleScan] Unknown mode: {mode}")
            return TriggerSensorScanResponse(success=False)

    # ==================== Pixel to 3D Conversion ====================

    def pixel_to_3d_callback(self, req):
        """
        픽셀 좌표를 로봇 base frame 3D 좌표로 변환

        CF ADD_TOMATO에서 사용: 사용자가 클릭한 위치의 3D 좌표 획득
        3x3 윈도우 + median으로 depth 추출하여 안정성 확보

        Args:
            req: PixelTo3D request (robot_name, pixel_x, pixel_y)

        Returns:
            PixelTo3DResponse (success, message, x, y, z)
        """
        try:
            pixel_x = req.pixel_x
            pixel_y = req.pixel_y

            rospy.loginfo(f"[PixelTo3D] Converting pixel ({pixel_x}, {pixel_y}) to 3D")

            # Depth 이미지 체크
            if self.latest_depth_msg is None:
                rospy.logwarn("[PixelTo3D] No depth image available")
                return PixelTo3DResponse(
                    success=False,
                    message="No depth image available",
                    x=0.0, y=0.0, z=0.0
                )

            # Depth 이미지 변환
            cv_image_depth = self.cv_bridge.imgmsg_to_cv2(self.latest_depth_msg)
            img_height, img_width = cv_image_depth.shape[:2]

            # 범위 체크
            if pixel_x < 0 or pixel_x >= img_width or pixel_y < 0 or pixel_y >= img_height:
                rospy.logwarn(f"[PixelTo3D] Pixel ({pixel_x}, {pixel_y}) out of bounds ({img_width}x{img_height})")
                return PixelTo3DResponse(
                    success=False,
                    message=f"Pixel out of bounds",
                    x=0.0, y=0.0, z=0.0
                )

            # === Depth 계산 ===
            # 사용자가 클릭한 단일 픽셀의 depth를 추출합니다.
            # perform_yolo_cam1_callback과 다른 접근 방식:
            # - YOLO callback: bounding box 전체 영역 + mean (토마토 중심 거리 추정)
            # - PixelTo3D: 3x3 윈도우 + median (클릭한 정확한 위치의 depth)
            #
            # 3x3 윈도우를 사용하는 이유:
            # - 사용자가 토마토 가장자리를 클릭할 수 있음 (depth 불안정 영역)
            # - 단일 픽셀보다 주변 픽셀을 포함하면 더 안정적
            # - median 사용으로 이상치(배경 depth 등) 제거

            window_size = 3
            half = window_size // 2

            y_min = max(0, int(pixel_y) - half)
            y_max = min(img_height, int(pixel_y) + half + 1)
            x_min = max(0, int(pixel_x) - half)
            x_max = min(img_width, int(pixel_x) + half + 1)

            depth_window = cv_image_depth[y_min:y_max, x_min:x_max].flatten()

            # 유효 depth 필터링: 100mm ~ 10000mm (너무 가깝거나 먼 값 제외)
            valid_depths = depth_window[(depth_window > 100) & (depth_window < 10000)]

            if len(valid_depths) == 0:
                rospy.logwarn(f"[PixelTo3D] No valid depth at pixel ({pixel_x}, {pixel_y})")
                return PixelTo3DResponse(
                    success=False,
                    message="No valid depth at this pixel",
                    x=0.0, y=0.0, z=0.0
                )

            # Median 사용: 이상치(가장자리의 배경 depth 등) 제거에 효과적
            # (cf. YOLO callback에서는 bounding box 전체 영역 + mean 사용)
            depth = np.median(valid_depths) * 0.001  # mm → m

            rospy.loginfo(f"[PixelTo3D] Depth at pixel: {depth:.3f}m (from {len(valid_depths)} valid samples)")

            # === 카메라 좌표계 → 3D 좌표 변환 ===
            # Pinhole 카메라 모델: x = (u - cx) * z / fx, y = (v - cy) * z / fy
            x_cam = (pixel_x - self.rgb_intrinsic["cx"]) * depth / self.rgb_intrinsic["fx"]
            y_cam = (pixel_y - self.rgb_intrinsic["cy"]) * depth / self.rgb_intrinsic["fy"]
            z_cam = depth

            # Robot base frame으로 TF 변환
            pos_robot = self.transform_camera_to_robot_base_cam1([x_cam, y_cam, z_cam])

            # TF 실패 체크 (0,0,0 반환 시)
            if pos_robot["x"] == 0.0 and pos_robot["y"] == 0.0 and pos_robot["z"] == 0.0:
                rospy.logwarn("[PixelTo3D] TF transformation failed")
                return PixelTo3DResponse(
                    success=False,
                    message="TF transformation failed",
                    x=0.0, y=0.0, z=0.0
                )

            rospy.loginfo(f"[PixelTo3D] Result: ({pos_robot['x']:.3f}, {pos_robot['y']:.3f}, {pos_robot['z']:.3f})")

            return PixelTo3DResponse(
                success=True,
                message="OK",
                x=pos_robot["x"],
                y=pos_robot["y"],
                z=pos_robot["z"]
            )

        except Exception as e:
            rospy.logerr(f"[PixelTo3D] Error: {e}")
            return PixelTo3DResponse(
                success=False,
                message=str(e),
                x=0.0, y=0.0, z=0.0
            )

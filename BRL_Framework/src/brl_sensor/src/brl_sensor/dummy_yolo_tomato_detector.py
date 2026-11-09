#!/usr/bin/env python3
"""카메라와 실제 로봇 없이 고정된 토마토 검출/스캔 결과를 제공한다."""

import threading

import rospy
from geometry_msgs.msg import Point, Pose2D

from brl_interfaces.msg import Object as ObjectMsg
from brl_interfaces.msg import Robot as RobotMsg
from brl_interfaces.srv import (
    TriggerSensorScan,
    TriggerSensorScanResponse,
    UpdateObject,
    UpdateObjectRequest,
    UpdateRobot,
    UpdateRobotRequest,
)


class DummyYOLOTomatoDetector:
    """YOLOTomatoDetector와 같은 scan callback을 제공하는 고정 시나리오 더미."""

    LOCATION_NAME = "Stem_01"
    DETECTIONS = (
        {
            "name": "tomato1",
            "position": (0.45, 0.10, 0.85),
            "bounding_box": (100.0, 120.0, 220.0, 260.0),
            "tomato_class": 0,
            "confidence": 0.95,
        },
        {
            "name": "tomato2",
            "position": (0.55, -0.10, 0.90),
            "bounding_box": (300.0, 130.0, 420.0, 270.0),
            "tomato_class": 0,
            "confidence": 0.70,
        },
    )
    SCAN_CLASSES = {"tomato1": 0, "tomato2": 2}

    def __init__(self, robot_name, object_update_client):
        self.robot_name = robot_name
        self.object_update_client = object_update_client
        self.single_scan_active = False
        self.single_scan_target = ""
        self.lock = threading.Lock()
        self.latest_detection_result_cam1 = []

    @staticmethod
    def _make_object(detection, current_location_name, held_by_robot_name=""):
        x1, y1, x2, y2 = detection["bounding_box"]
        object_msg = ObjectMsg()
        object_msg.name = detection["name"]
        object_msg.type = "tomato"
        object_msg.status = "detected"
        object_msg.position = Point(*detection["position"])
        object_msg.current_location_name = current_location_name
        object_msg.held_by_robot_name = held_by_robot_name
        object_msg.properties = [
            f"tomato_class:{detection['tomato_class']}",
            f"confidence:{detection['confidence']:.3f}",
            f"bounding_box:[{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}]",
            f"pixel_center:{(x1 + x2) / 2.0:.1f},{(y1 + y2) / 2.0:.1f}",
        ]
        return object_msg

    def _update_object(self, object_msg):
        request = UpdateObjectRequest()
        request.object_data = object_msg
        return self.object_update_client(request)

    def sensor_scan_callback(self, req):
        """Stem_01에서 ripe 토마토 두 개를 검출하고 KG에 저장한다."""
        rospy.loginfo(
            "[%s] Dummy sensor scan: requested='%s', using='%s'",
            self.robot_name,
            req.target_location,
            self.LOCATION_NAME,
        )

        self.latest_detection_result_cam1 = []
        for detection in self.DETECTIONS:
            object_msg = self._make_object(detection, self.LOCATION_NAME)
            response = self._update_object(object_msg)
            if not response.success:
                return TriggerSensorScanResponse(success=False, message=response.message)
            self.latest_detection_result_cam1.append(object_msg)
            rospy.loginfo(
                "[DummyYOLO] Detected %s: class=ripe, confidence=%.3f",
                detection["name"],
                detection["confidence"],
            )

        return TriggerSensorScanResponse(
            success=True,
            message="Detected tomato1=ripe(0.95), tomato2=ripe(0.70) at Stem_01",
        )

    def scan_single_tomato(self, req, mode):
        """단일 스캔 결과를 tomato1=ripe, tomato2=rotten으로 갱신한다."""
        with self.lock:
            if mode == "start":
                self.single_scan_active = True
                self.single_scan_target = req.target_object
                rospy.loginfo("[DummyYOLO] Single scan started: %s", req.target_object)
                return TriggerSensorScanResponse(success=True, message="Dummy single scan started")

            if mode != "end":
                return TriggerSensorScanResponse(success=False, message=f"Unknown mode: {mode}")

            if not self.single_scan_active:
                return TriggerSensorScanResponse(success=True, message="Dummy single scan is not running")

            tomato_name = req.target_object or self.single_scan_target
            if tomato_name not in self.SCAN_CLASSES:
                self.single_scan_active = False
                return TriggerSensorScanResponse(success=False, message=f"Unknown tomato: {tomato_name}")

            detection = dict(next(item for item in self.DETECTIONS if item["name"] == tomato_name))
            detection["tomato_class"] = self.SCAN_CLASSES[tomato_name]
            detection["confidence"] = 1.0
            object_msg = self._make_object(detection, "", self.robot_name)
            response = self._update_object(object_msg)
            self.single_scan_active = False
            self.single_scan_target = ""

            class_name = "ripe" if detection["tomato_class"] == 0 else "rotten"
            rospy.loginfo("[DummyYOLO] Single scan completed: %s=%s", tomato_name, class_name)
            return TriggerSensorScanResponse(success=response.success, message=response.message)


def update_dummy_robot(robot_name, robot_update_client):
    """더미 로봇을 Stem_01에 등록한다."""
    robot_msg = RobotMsg()
    robot_msg.name = robot_name
    robot_msg.type = "harvester"
    robot_msg.status = "active"
    robot_msg.position = Pose2D(x=0.0, y=0.0, theta=0.0)
    robot_msg.current_location_name = DummyYOLOTomatoDetector.LOCATION_NAME
    robot_msg.capabilities = ["NavigateTo", "Harvest"]

    request = UpdateRobotRequest()
    request.robot_data = robot_msg
    return robot_update_client(request)


def main():
    rospy.init_node("dummy_yolo_tomato_detector")
    robot_name = rospy.get_param("~robot_name", "brl_robot")

    rospy.wait_for_service("/knowledge_graph_node/robot/update")
    rospy.wait_for_service("/knowledge_graph_node/object/update")
    robot_update_client = rospy.ServiceProxy("/knowledge_graph_node/robot/update", UpdateRobot)
    object_update_client = rospy.ServiceProxy("/knowledge_graph_node/object/update", UpdateObject)

    update_dummy_robot(robot_name, robot_update_client)
    detector = DummyYOLOTomatoDetector(robot_name, object_update_client)

    rospy.Service(f"/{robot_name}/trigger_sensor_scan", TriggerSensorScan, detector.sensor_scan_callback)
    rospy.Service(
        f"/{robot_name}/start_single_tomato_scan",
        TriggerSensorScan,
        lambda req: detector.scan_single_tomato(req, "start"),
    )
    rospy.Service(
        f"/{robot_name}/end_single_tomato_scan",
        TriggerSensorScan,
        lambda req: detector.scan_single_tomato(req, "end"),
    )

    rospy.loginfo(
        "[DummyYOLO] Ready: robot='%s', location='%s'",
        robot_name,
        DummyYOLOTomatoDetector.LOCATION_NAME,
    )
    rospy.spin()


if __name__ == "__main__":
    main()

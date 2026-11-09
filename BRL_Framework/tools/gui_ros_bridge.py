#!/usr/bin/env python3
"""Read-only ROS telemetry for the desktop GUI, isolated from Qt's event loop."""

import argparse
import json
import os
import threading
import time
import xmlrpc.client

import rospy
from brl_interfaces.msg import SceneGraphMsg
from std_msgs.msg import String

output_lock = threading.Lock()


def emit(event):
    with output_lock:
        print("BRL_GUI:" + json.dumps(event, ensure_ascii=False), flush=True)


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection


def scene_callback(message):
    objects = []
    for obj in message.objects:
        properties = dict(prop.split(":", 1) for prop in obj.properties if ":" in prop)
        tomato_class = properties.get("tomato_class", "")
        objects.append({"name": obj.name, "status": obj.status,
                        "location": obj.current_location_name, "holder": obj.held_by_robot_name,
                        "class": {"0": "ripe", "1": "unripe", "2": "rotten"}.get(tomato_class, tomato_class)})
    emit({"type": "scene", "environment": message.environment.name,
          "robots": [{"name": robot.name, "status": robot.status,
                      "location": robot.current_location_name, "held_objects": list(robot.held_objects)}
                     for robot in message.robots], "objects": objects})

def targets_callback(message):
    try:
        targets = json.loads(message.data)
        if isinstance(targets, list):
            emit({"type": "targets", "targets": targets})
    except ValueError:
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-name", default="brl_robot")
    args = parser.parse_args()
    master = xmlrpc.client.ServerProxy(os.environ.get("ROS_MASTER_URI", "http://localhost:11311"),
                                      transport=TimeoutTransport())
    while True:
        try:
            if master.getPid("brl_framework_gui")[0] == 1:
                break
        except (OSError, xmlrpc.client.Error):
            pass
        emit({"type": "connection", "connected": False, "planner_ready": False})
        time.sleep(1.0)
    rospy.init_node("brl_framework_gui", anonymous=True, disable_signals=True)
    subscribers = [
        rospy.Subscriber("/scene_graph", SceneGraphMsg, scene_callback, queue_size=1),
        rospy.Subscriber("/pomdp_planner/status", String,
                         lambda message: emit({"type": "status", "status": message.data}), queue_size=1),
        rospy.Subscriber("/pomdp_planner/current_action", String,
                         lambda message: emit({"type": "action", "action": message.data}), queue_size=1),
        rospy.Subscriber("/%s/selected_targets" % args.robot_name, String, targets_callback, queue_size=1),
    ]
    while not rospy.is_shutdown():
        try:
            services = master.getSystemState("brl_framework_gui")[2][2]
            names = {name for name, providers in services}
            emit({"type": "connection", "connected": True,
                  "planner_ready": {"/generate_task_plan", "/task_planner/start"}.issubset(names)})
        except (OSError, xmlrpc.client.Error, IndexError):
            emit({"type": "connection", "connected": False, "planner_ready": False})
            break  # A fresh process re-registers subscriptions after master restart.
        time.sleep(1.0)
    rospy.signal_shutdown("GUI status receiver stopped")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

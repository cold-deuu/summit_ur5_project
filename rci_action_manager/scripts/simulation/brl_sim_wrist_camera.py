#!/usr/bin/env python3
"""BRL sim wrist camera (camera2): renders the MuJoCo scene from hand_camera_link.

Separate process from moveit_mujoco_backend.py: loads the same scene, adds a camera to the
mount body (MjSpec), copies the backend's /mujoco/joint_states into qpos and publishes what
the D435 color stream would: /brl_robot/camera2/color/image_raw (rgb8) + camera_info.

Mount: BRL model hand_camera_link (camera_bracket + (0.0813, 0, 0.0239)), looking along the
bracket z (gripper approach). Image axes follow BRL visual_servoing.py: at the pick
orientation image right = -x and image up = +z of robot_base_footprint (its centering moves
+x for a target left of the window and +z for a target above it). The BRL model has no
camera tilt, so the camera axis is parallel to the gripper axis, 8 cm above it.
Intrinsics: lab D435 color 640x480, fy = 604.9 -> fovy 43.3 deg, principal point centered.
Depth: same camera, so already aligned to color; 16UC1 mm like realsense aligned_depth_to_color,
0 outside [~depth_min, ~depth_max] (D435 has no depth very close). Also publishes the static
TF mount -> ~frame_id (optical: x right, y down, z forward), i.e. the sim "hand-eye" result.
Needs an X display (MUJOCO_GL=glfw, hidden window); the installed PyOpenGL has no EGL device ext.
"""
import os
os.environ.setdefault("MUJOCO_GL", "glfw")
import math
import threading

import mujoco
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, JointState

BASE = {"v_x_joint": "base_x_slide_joint", "v_y_joint": "base_y_slide_joint", "v_t_joint": "base_z_hinge_joint"}


def main():
    rospy.init_node("brl_sim_wrist_camera")
    model_file = rospy.get_param("~model_file")
    mount = rospy.get_param("~mount_body", "hand_camera_link")
    pos = rospy.get_param("~pos", [0.0, 0.0, 0.0])
    xyaxes = rospy.get_param("~xyaxes", [0, 1, 0, 1, 0, 0])  # camera x (image right), y (image up) in mount
    width, height = int(rospy.get_param("~width", 640)), int(rospy.get_param("~height", 480))
    fovy = float(rospy.get_param("~fovy_deg", 43.3))
    rate_hz = float(rospy.get_param("~rate", 15.0))
    frame_id = rospy.get_param("~frame_id", "camera2_color_optical_frame")
    prefix = rospy.get_param("~topic_prefix", "/brl_robot/camera2/color")
    depth_topic = rospy.get_param("~depth_topic", "/brl_robot/camera2/aligned_depth_to_color/image_raw")
    depth_min, depth_max = float(rospy.get_param("~depth_min", 0.15)), float(rospy.get_param("~depth_max", 3.0))

    spec = mujoco.MjSpec()
    spec.from_file(model_file)
    body = spec.find_body(mount)
    if body is None:
        raise rospy.ROSException("mount body %s not in %s" % (mount, model_file))
    cam = body.add_camera()
    cam.name = "brl_wrist_camera"
    cam.pos = pos
    x = np.asarray(xyaxes[:3], float); y = np.asarray(xyaxes[3:], float)
    x /= np.linalg.norm(x); y -= x * (x @ y); y /= np.linalg.norm(y)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.column_stack([x, y, np.cross(x, y)]).flatten())
    cam.quat = quat
    cam.fovy = fovy
    model = spec.compile()
    data = mujoco.MjData(model)
    qadr = {model.joint(i).name: int(model.jnt_qposadr[i]) for i in range(model.njnt)}
    renderer = mujoco.Renderer(model, height, width)
    depth_renderer = mujoco.Renderer(model, height, width)
    depth_renderer.enable_depth_rendering()

    # Optical frame in the mount: x = image right = camera x, y = image down = -camera y, z = forward.
    optical = np.column_stack([x, -y, -np.cross(x, y)])
    oq = np.zeros(4)
    mujoco.mju_mat2Quat(oq, optical.flatten())
    mount_tf = TransformStamped()
    mount_tf.header.stamp = rospy.Time.now()
    mount_tf.header.frame_id = mount
    mount_tf.child_frame_id = frame_id
    mount_tf.transform.translation.x, mount_tf.transform.translation.y, mount_tf.transform.translation.z = pos
    mount_tf.transform.rotation.w, mount_tf.transform.rotation.x, mount_tf.transform.rotation.y, mount_tf.transform.rotation.z = oq
    static_tf = tf2_ros.StaticTransformBroadcaster()
    static_tf.sendTransform(mount_tf)

    latest = {}
    lock = threading.Lock()

    def joints(msg):
        with lock:
            for name, value in zip(msg.name, msg.position):
                name = BASE.get(name, name[5:] if name.startswith("ur5e_") else name)
                if name in qadr:
                    latest[name] = value
    rospy.Subscriber("/mujoco/joint_states", JointState, joints, queue_size=1)

    image_pub = rospy.Publisher(prefix + "/image_raw", Image, queue_size=1)
    info_pub = rospy.Publisher(prefix + "/camera_info", CameraInfo, queue_size=1)
    depth_pub = rospy.Publisher(depth_topic, Image, queue_size=1)
    f = (height / 2.0) / math.tan(math.radians(fovy) / 2.0)
    info = CameraInfo(width=width, height=height, distortion_model="plumb_bob", D=[0.0] * 5,
                      K=[f, 0, width / 2.0, 0, f, height / 2.0, 0, 0, 1],
                      R=[1, 0, 0, 0, 1, 0, 0, 0, 1], P=[f, 0, width / 2.0, 0, 0, f, height / 2.0, 0, 0, 0, 1, 0])
    info.header.frame_id = frame_id
    rospy.loginfo("[brl_sim_wrist_camera] %s on %s, %dx%d fovy %.1f (f=%.1f px) @ %.0f Hz",
                  prefix + "/image_raw", mount, width, height, fovy, f, rate_hz)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        with lock:
            if not latest:
                rate.sleep()
                continue
            for name, value in latest.items():
                data.qpos[qadr[name]] = value
        mujoco.mj_kinematics(model, data)
        mujoco.mj_camlight(model, data)
        renderer.update_scene(data, camera="brl_wrist_camera")
        pixels = renderer.render()
        depth_renderer.update_scene(data, camera="brl_wrist_camera")
        depth = depth_renderer.render()
        depth_mm = np.where((depth >= depth_min) & (depth <= depth_max), depth * 1000.0, 0.0).astype(np.uint16)
        stamp = rospy.Time.now()
        msg = Image(height=height, width=width, encoding="rgb8", is_bigendian=0, step=width * 3,
                    data=pixels.tobytes())
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        info.header.stamp = stamp
        image_pub.publish(msg)
        depth_msg = Image(height=height, width=width, encoding="16UC1", is_bigendian=0, step=width * 2,
                          data=depth_mm.tobytes())
        depth_msg.header = msg.header
        depth_pub.publish(depth_msg)
        info_pub.publish(info)
        rate.sleep()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Exercise the real ROS action controller and record offscreen MuJoCo replay.

Run after sourcing devel/setup.bash, with MUJOCO_GL=egl. Uses an isolated ROS
master (11321 by default); needs cv2, imageio_ffmpeg and working EGL bindings.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

import actionlib
import cv2
import imageio_ffmpeg
import mujoco
import numpy as np
import rospy
from rci_action_manager.msg import JointPostureAction, JointPostureGoal, SE3Action, SE3Goal

HOME = np.array([np.pi, -1.57, 1.57, -1.57, -1.57, 0.0])
JOINT_TARGET = HOME + np.array([-0.35, 0.30, -0.25, -0.20, 0.20, 0.30])
SE3_OFFSET = np.array([0.08, 0.05, 0.04])


def rotation(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def fk_setup(path):
    root = ET.parse(path).getroot()
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    chain, child = [], "ur5e_wrist_3_link"
    while child in by_child:
        joint = by_child[child]
        chain.append(joint)
        child = joint.find("parent").get("link")
    chain.reverse()
    names = ["ur5e_shoulder_pan_joint", "ur5e_shoulder_lift_joint", "ur5e_elbow_joint",
             "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"]

    def fk(q):
        pose = np.eye(4)
        for joint in chain:
            origin = joint.find("origin")
            step = np.eye(4)
            if origin is not None:
                step[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                step[:3, :3] = rotation(np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
            pose = pose @ step
            if joint.get("name") in names:
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                axis /= np.linalg.norm(axis)
                a = q[names.index(joint.get("name"))]
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                step = np.eye(4)
                step[:3, :3] = np.eye(3) + np.sin(a)*skew + (1-np.cos(a))*(skew @ skew)
                pose = pose @ step
        return pose
    return fk


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--port", type=int, default=11321)
    parser.add_argument("--force-pulse", action="store_true",
                        help="Apply 6 N at the wrist origin for 0.2 s during each action")
    parser.add_argument("--baseline", help="Unforced run's results.json, required with --force-pulse")
    args = parser.parse_args()
    if args.force_pulse and not args.baseline:
        parser.error("--force-pulse requires --baseline")
    baseline = json.loads(Path(args.baseline).read_text()) if args.baseline else None
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    workspace = Path(__file__).resolve().parents[4]
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % args.port
    os.environ["ROS_HOSTNAME"] = "127.0.0.1"
    master_log = open(out / "roscore.log", "w")
    master = subprocess.Popen(["roscore", "-p", str(args.port)], stdout=master_log, stderr=subprocess.STDOUT)
    controller = None
    stop = threading.Event()
    worker = None
    try:
        for _ in range(100):
            try:
                if xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]).getPid("/headless_verify")[0] == 1:
                    break
            except OSError:
                pass
            time.sleep(0.1)
        else:
            raise RuntimeError("Isolated ROS master did not start")
        # Reuse the simulator's actual initialization, subscriptions and publications.
        spec = importlib.util.spec_from_file_location("project_simulator", Path(__file__).with_name("mujoco_launch.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sim = module.mujocoSim()
        rospy.set_param("/whole_body", False)
        sim.model.opt.gravity[:] = 0  # Same setting as the existing simulator.
        lock = threading.RLock()
        samples = []
        capture = False
        sent = float("inf")
        wrist_body = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")

        def loop():
            rate = rospy.Rate(100)
            try:
                while not stop.is_set() and not rospy.is_shutdown():
                    with lock:
                        sim.data.xfrc_applied[:] = 0
                        if args.force_pulse and capture and 1.8 <= time.monotonic()-sent < 2.0:
                            sim.data.xfrc_applied[wrist_body, 0] = 6.0
                        sim.joint_states_publish(sim.data.qpos, sim.data.qvel)
                        sim.ft_sensor_publish()
                        for _ in range(round(0.01 / sim.model.opt.timestep)):
                            mujoco.mj_step(sim.model, sim.data)
                        if capture:
                            samples.append((time.monotonic(), sim.data.qpos.copy(),
                                            sim.data.qvel.copy(), sim.data.ctrl.copy(), sim.data.ncon, sim.ft_sensor.copy(),
                                            sim.data.xfrc_applied[wrist_body].copy()))
                    rate.sleep()
            except BaseException as exc:
                print("Simulation loop failed:", repr(exc), flush=True)
                stop.set()

        fk = fk_setup(workspace / "src/robot_description/summit_xl_description/summit_ur_manip_control.urdf")
        results = []
        for kind in ("joint_posture", "se3"):
            with lock:
                mujoco.mj_resetData(sim.model, sim.data)
                sim.data.qpos[:] = HOME
                sim.data.ctrl[:] = HOME
                mujoco.mj_forward(sim.model, sim.data)
            log_path = out / (kind + "_controller.log")
            log = open(log_path, "w")
            controller = subprocess.Popen(
                [str(workspace / "devel/lib/rci_action_manager/rci_action_manager_simulation")],
                stdout=log, stderr=subprocess.STDOUT)
            if worker is None:
                worker = threading.Thread(target=loop, daemon=True)
                worker.start()
            joint = actionlib.SimpleActionClient("rci_action_manager/jointControl", JointPostureAction)
            se3 = actionlib.SimpleActionClient("rci_action_manager/se3Control", SE3Action)
            if not joint.wait_for_server(rospy.Duration(15)) or not se3.wait_for_server(rospy.Duration(15)):
                raise RuntimeError("Action server unavailable: " + log_path.read_text())
            # Wait for joint-state/command transport and startup sensor transients.
            time.sleep(1.0)
            # Establish the client's home via an actual action, not only model initialization.
            home = JointPostureGoal()
            home.target_joints.position = HOME.tolist()
            home.duration = 2.0
            joint.send_goal(home)
            if not joint.wait_for_result(rospy.Duration(10)) or joint.get_state() != 3:
                raise RuntimeError("Home action failed")
            with lock:
                home_error = float(np.max(np.abs(sim.data.qpos - HOME)))
                initial_pose = fk(sim.data.qpos.copy())
                samples.clear()
                sent = float("inf")
                capture = True
            time.sleep(1)
            if kind == "joint_posture":
                client = joint
                goal = JointPostureGoal()
                goal.target_joints.position = JOINT_TARGET.tolist()
                goal.duration = 5
            else:
                client = se3
                goal = SE3Goal()
                goal.target_pose.position.x, goal.target_pose.position.y, goal.target_pose.position.z = SE3_OFFSET
                goal.target_pose.orientation.w = 1
                goal.relative = True
                goal.wholebody = False
                goal.duration = 5
            sent = time.monotonic()
            client.send_goal(goal)
            completed = client.wait_for_result(rospy.Duration(18))
            finished = time.monotonic()
            status = client.get_state()
            payload = client.get_result()
            with lock:
                final_q = sim.data.qpos.copy()
            time.sleep(1)
            with lock:
                capture = False
                data = list(samples)
            controller.terminate()
            controller.wait(timeout=10)
            controller = None
            log.close()
            q = np.array([s[1] for s in data])
            v = np.array([s[2] for s in data])
            ctrl = np.array([s[3] for s in data])
            timestamps = np.array([s[0] for s in data])
            np.savez_compressed(out / (kind + "_states.npz"),
                                time=timestamps-timestamps[0], qpos=q, qvel=v, ctrl=ctrl,
                                contacts=np.array([s[4] for s in data]),
                                sensor_wrench=np.array([s[5] for s in data]),
                                applied_wrench_world=np.array([s[6] for s in data]))
            pose = fk(final_q)
            goal_pose = initial_pose.copy()
            goal_pose[:3, 3] += SE3_OFFSET
            position_error = float(np.linalg.norm(pose[:3, 3]-goal_pose[:3, 3]))
            orientation_error = float(np.arccos(np.clip((np.trace(goal_pose[:3, :3].T @ pose[:3, :3])-1)/2, -1, 1)))
            numeric = bool(np.isfinite(q).all() and np.isfinite(v).all() and np.isfinite(ctrl).all())
            failure_count = log_path.read_text().count("HQP solve failed")
            result = dict(action=kind, action_status=int(status), wait_completed=bool(completed),
                          result_is_completed=bool(payload.is_completed) if payload else None,
                          home_max_joint_error_rad=home_error,
                          elapsed_action_seconds=finished-sent, samples=len(data),
                          max_abs_joint_velocity_rad_s=float(np.max(np.abs(v))),
                          max_contacts=max(s[4] for s in data),
                          finite_states_and_commands=numeric, hqp_failure_log_count=failure_count,
                          start_q=q[0].tolist(), final_q=final_q.tolist())
            if kind == "joint_posture":
                result["target_q"] = JOINT_TARGET.tolist()
                result["max_joint_error_rad"] = float(np.max(np.abs(final_q-JOINT_TARGET)))
                error_ok = result["max_joint_error_rad"] < 0.01
            else:
                result.update(relative_translation_m=SE3_OFFSET.tolist(),
                              position_error_m=position_error, orientation_error_rad=orientation_error,
                              initial_position_m=initial_pose[:3, 3].tolist(),
                              target_position_m=goal_pose[:3, 3].tolist(),
                              final_position_m=pose[:3, 3].tolist())
                error_ok = position_error < 0.005 and orientation_error < 0.02
            if args.force_pulse:
                nominal = next(r for r in baseline["results"] if r["action"] == kind)
                delta = pose[:3, 3] - fk(np.array(nominal["final_q"]))[:3, 3]
                result["displacement_vs_unforced_m"] = delta.tolist()
                result["peak_sensor_force_N"] = float(np.max(np.linalg.norm(
                    np.array([s[5][:3] for s in data]), axis=1)))
                result["force_pulse_samples"] = int(sum(s[6][0] != 0 for s in data))
                # Arm-only MuJoCo and the controller world differ by a pi yaw:
                # positive simulator X force must produce negative controller X displacement.
                result["final_max_joint_velocity_rad_s"] = float(np.max(np.abs(v[-50:])))
                error_ok = (delta[0] < -0.005 and result["force_pulse_samples"] > 0
                            and result["final_max_joint_velocity_rad_s"] < 0.01)
            result["passed"] = bool(completed and status == 3 and numeric and failure_count == 0 and home_error < .01 and error_ok)
            results.append(result)
            (out / "results.json").write_text(json.dumps(dict(
                mode="manipulator_only", whole_body=False, gravity=[0, 0, 0],
                home_source="mm_client.py do_home", sca_enabled=False,
                external_force_pulse=bool(args.force_pulse),
                validation="Real ROS actions; measured state errors, not action status alone",
                results=results), indent=2))
            print(json.dumps(result), flush=True)

            # Render recorded physical states afterwards, keeping rendering out of the control loop.
            sim.model.vis.global_.offwidth = 960
            sim.model.vis.global_.offheight = 720
            renderer = mujoco.Renderer(sim.model, height=720, width=960)
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [.15, 0, .35]
            camera.distance = 1.7
            camera.azimuth = 130
            camera.elevation = -20
            replay = mujoco.MjData(sim.model)
            video_path = out / (kind + ".mp4")
            fps = 30
            writer = imageio_ffmpeg.write_frames(str(video_path), (960, 720), fps=fps,
                                                 codec="libx264", pix_fmt_out="yuv420p")
            writer.send(None)
            frames = 0
            try:
                for t in np.arange(0, timestamps[-1]-timestamps[0], 1/fps):
                    idx = min(np.searchsorted(timestamps-timestamps[0], t), len(data)-1)
                    replay.qpos[:] = q[idx]
                    mujoco.mj_forward(sim.model, replay)
                    renderer.update_scene(replay, camera=camera)
                    frame = renderer.render().copy()
                    phase = "HOME" if timestamps[0]+t < sent else ("ACTION" if timestamps[0]+t < finished else "SETTLED")
                    cv2.rectangle(frame, (0, 0), (960, 95), (20, 20, 20), -1)
                    if args.force_pulse and 1.8 <= timestamps[0]+t-sent < 2.0:
                        phase += " + 6N PUSH"
                    cv2.putText(frame, kind.upper()+" | "+phase, (20, 35),
                                cv2.FONT_HERSHEY_SIMPLEX, .8, (245, 245, 245), 2)
                    cv2.putText(frame, "Headless MuJoCo | ROS HQP | arm only | t=%.2fs" % t,
                                (20, 70), cv2.FONT_HERSHEY_SIMPLEX, .6, (200, 220, 230), 1)
                    writer.send(frame)
                    if frames == 0:
                        cv2.imwrite(str(out / (kind+"_preview.png")), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                    frames += 1
            finally:
                writer.close()
                renderer.close()
            print("Saved", video_path, frames, "frames", flush=True)
        if not all(r["passed"] for r in results):
            raise RuntimeError("One or more action validations failed; see results.json")
    finally:
        stop.set()
        if worker is not None:
            worker.join(timeout=3)
        if controller is not None and controller.poll() is None:
            controller.terminate()
            controller.wait(timeout=10)
        master.terminate()
        master.wait(timeout=15)
        master_log.close()


if __name__ == "__main__":
    main()

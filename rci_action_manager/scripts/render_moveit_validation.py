#!/usr/bin/env python3
"""Render the measured status_history.npz after testing; never drives the robot."""
import argparse
import json
import os
os.environ.setdefault("MUJOCO_GL","egl")
from pathlib import Path
import cv2
import imageio_ffmpeg
import mujoco
import numpy as np
import rospkg

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("directory")
    parser.add_argument("--tracking-only",action="store_true")
    args=parser.parse_args()
    root=Path(args.directory)
    scenario=(root/"scenario_events.json").exists()
    events=json.loads((root/"scenario_events.json").read_text()) if scenario else []
    scenario_info=json.loads((root/"scenario.json").read_text()) if scenario else {}
    data=np.load(root/"status_history.npz")
    results=json.loads((root/"results.json").read_text())
    tracking_admittance=all(r.get("tracking_admittance",True) for r in results)
    t=data["time"]-data["time"][0]
    tracked=data["state"]=="TRACKING"
    starts=np.where(tracked & np.r_[True,~tracked[:-1]])[0]
    ends=np.where(tracked & np.r_[~tracked[1:],True])[0]
    labels=[]
    names={"return_to_test_home":"RETURN TO HOME","joint_plan":"JOINT GOAL",
           "pose_plan":"POSE GOAL","cartesian_plan":"CARTESIAN PATH",
           "replace_while_tracking":"TRAJECTORY REPLACEMENT",
           "shuffled_joint_names":"SHUFFLED JOINT NAMES","future_stamp":"FUTURE START",
           "tracking_force_return":"TRACKING + EXTERNAL FORCE"}
    for result in results:
        if result["test"] in names or "trajectory_id" in result:
            labels.append(names.get(result["test"],result["test"].replace("_"," ").upper()))
    assert len(labels)==len(starts),(labels,starts)
    spans=[(float(t[a]),float(t[b]),label) for a,b,label in zip(starts,ends,labels)]
    holding_start=float(t[ends[-2]]) if len(ends)>1 else 0
    holding_end=float(t[starts[-1]])
    norm=np.linalg.norm(data["displacement"][:,:3],axis=1)
    pulse_onsets=np.where((norm>.002)&np.r_[True,norm[:-1]<=.002]&
                          (t>holding_start)&(t<holding_end))[0]
    hold_boundary=float(t[pulse_onsets[1]])-1 if len(pulse_onsets)>1 else holding_start+7
    plateau=norm[(t>hold_boundary+2)&(t<hold_boundary+4)]
    baseline=float(np.median(plateau)) if len(plateau) else .04
    third=np.where((t>hold_boundary+4)&(t<holding_end)&(norm>baseline+.0015))[0]
    switch_boundary=float(t[third[0]])-.1 if len(third) else holding_end-5
    model_path=Path(rospkg.RosPack().get_path("summit_xl_description")).parent/"universal_robots_ur5e/scene.xml"
    configuration=json.loads((root/"model_config.json").read_text()) if (root/"model_config.json").exists() else {}
    if configuration.get("model_file"):model_path=Path(configuration["model_file"])
    mobile=configuration.get("mobile",False)
    model=mujoco.MjModel.from_xml_path(str(model_path))
    arm_addresses=[int(model.jnt_qposadr[model.joint(n).id]) for n in
        ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]]
    model.vis.global_.offwidth=960;model.vis.global_.offheight=720
    renderer=mujoco.Renderer(model,720,960)
    actual=mujoco.MjData(model);reference=mujoco.MjData(model)
    equilibrium=mujoco.MjData(model)
    if scenario:
        equilibrium.qpos[arm_addresses]=scenario_info["nominal_goal_q"]
        mujoco.mj_forward(model,equilibrium)
    camera=mujoco.MjvCamera()
    camera.lookat[:]=[-.15,0,.35];camera.distance=1.8;camera.azimuth=110;camera.elevation=-23
    if mobile:
        camera.lookat[:]=[.1,0,.65];camera.distance=2.8;camera.azimuth=125;camera.elevation=-20
    fps=30
    outputs=[
        ("tracking.mp4",0,holding_start+.4),
        ("admittance.mp4",max(0,holding_start-.3),float(t[-1])),
    ]
    if args.tracking_only:
        outputs=[("moveit_long_tracking.mp4",0,float(t[-1]))]
    if scenario:
        outputs=[("hold_track_push.mp4",0,float(t[-1]))]
    manifest=dict(source="measured ROS joint states, offscreen replay after testing",
                  fps=fps,resolution=[960,720],tracking_spans=spans,
                  spring_window=[holding_start,hold_boundary],
                  hold_window=[hold_boundary,switch_boundary],
                  mode_switch_window=[switch_boundary,holding_end])
    if scenario:
        manifest["scenario"]=scenario_info
        manifest["events"]=events
        for key in ["spring_window","hold_window","mode_switch_window"]:
            manifest.pop(key,None)
    (root/"video_manifest.json").write_text(json.dumps(manifest,indent=2))
    try:
        for filename,begin,end in outputs:
            writer=imageio_ffmpeg.write_frames(str(root/filename),(960,720),fps=fps,
                                               codec="libx264",pix_fmt_out="yuv420p")
            writer.send(None)
            count=0
            try:
                for time in np.arange(begin,end,1/fps):
                    i=min(np.searchsorted(t,time),len(t)-1)
                    actual.qpos[arm_addresses]=data["actual"][i];reference.qpos[arm_addresses]=data["desired"][i]
                    mujoco.mj_forward(model,actual);mujoco.mj_forward(model,reference)
                    event={}
                    if scenario:
                        stamp=float(data["time"][i])
                        for candidate in events:
                            if candidate["stamp"]<=stamp: event=candidate
                    renderer.update_scene(actual,camera=camera)
                    scene=renderer.scene
                    if scene.ngeom<len(scene.geoms):
                        point=scene.geoms[scene.ngeom]
                        mujoco.mjv_initGeom(point,mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([.012,.012,.012]),reference.site("ee_site").xpos,
                            np.eye(3).ravel(),np.array([.15,1,.3,.8],dtype=np.float32))
                        scene.ngeom+=1
                    if scenario and scene.ngeom<len(scene.geoms):
                        marker=scene.geoms[scene.ngeom]
                        mujoco.mjv_initGeom(marker,mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([.013,.013,.013]),equilibrium.site("ee_site").xpos,
                            np.eye(3).ravel(),np.array([.2,.5,1,.9],dtype=np.float32))
                        scene.ngeom+=1
                    if scenario and event.get("force_N",0)>0 and scene.ngeom<len(scene.geoms):
                        arrow=scene.geoms[scene.ngeom]
                        point=actual.body("wrist_3_link").xpos.copy()
                        mujoco.mjv_initGeom(arrow,mujoco.mjtGeom.mjGEOM_ARROW,
                            np.array([.01,.01,.18]),point,np.eye(3).ravel(),
                            np.array([1,.2,.15,1],dtype=np.float32))
                        mujoco.mjv_connector(arrow,mujoco.mjtGeom.mjGEOM_ARROW,.01,
                            point-np.array([.18,0,0]),point)
                        scene.ngeom+=1
                    frame=renderer.render().copy()
                    phase="HOLDING"
                    for a,b,label in spans:
                        if a<=time<=b:phase=label;break
                    if not scenario and not args.tracking_only and holding_start<=time<holding_end:
                        phase=("SPRING: PUSH / RELEASE / RETURN" if time<hold_boundary else
                               "HOLD: PUSH / RELEASE / OFFSET KEPT" if time<switch_boundary else
                               "RUNTIME MODE SWITCH / RESTORATION")
                    if scenario: phase=event.get("phase","INITIAL HOLD - 2 SECONDS")
                    cv2.rectangle(frame,(0,0),(960,120),(18,18,18),-1)
                    for y,text,scale,color in [
                        (32,("SUMMIT + UR5e | BASE FIXED | HQP + ADM" if mobile else "UR5e ONLY | HQP + ADMITTANCE"),.8,(245,245,245)),
                        (65,phase+((" | NO FORCE / ADM ON" if tracking_admittance else " | NO FORCE / ADM OFF") if args.tracking_only else ""),
                         .58,(180,230,255)),
                        (96,"t=%.2fs | %s | offset=%.1f mm | green=reference EE"%
                         (time,data["state"][i],norm[i]*1000),.52,(210,210,210))]:
                        cv2.putText(frame,text,(18,y),cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)
                    if scenario:
                        cv2.rectangle(frame,(0,676),(960,720),(18,18,18),-1)
                        cv2.putText(frame,"ADM ON | HOLD K=0 | blue=pre-push EE | red=external force",
                            (18,703),cv2.FONT_HERSHEY_SIMPLEX,.56,(220,230,245),1,cv2.LINE_AA)
                    writer.send(frame)
                    if count==0:
                        cv2.imwrite(str(root/filename.replace(".mp4","_preview.png")),cv2.cvtColor(frame,cv2.COLOR_RGB2BGR))
                    count+=1
            finally:writer.close()
            print(filename,count,"frames",flush=True)
    finally:renderer.close()

if __name__=="__main__":main()

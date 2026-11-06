#!/usr/bin/env python3
"""Offscreen replay of measured nine-axis action-test states."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json
from pathlib import Path
import cv2,imageio_ffmpeg,mujoco,numpy as np,rospkg
p=argparse.ArgumentParser();p.add_argument('directory');args=p.parse_args()
root=Path(args.directory);data=np.load(root/'states.npz');results=json.loads((root/'results.json').read_text())
model=mujoco.MjModel.from_xml_path(str(Path(rospkg.RosPack().get_path('summit_ur5e_moveit_config'))/'config/mobile_mujoco_scene.xml'))
model.vis.global_.offwidth=960;model.vis.global_.offheight=720
state=mujoco.MjData(model);renderer=mujoco.Renderer(model,720,960)
camera=mujoco.MjvCamera();camera.lookat[:]=[.15,0,.65];camera.distance=2.8;camera.azimuth=125;camera.elevation=-20
writer=imageio_ffmpeg.write_frames(str(root/'mobile_actions.mp4'),(960,720),fps=30,codec='libx264',pix_fmt_out='yuv420p');writer.send(None)
try:
    for stamp in np.arange(data['time'][0],data['time'][-1],1/30):
        i=min(np.searchsorted(data['time'],stamp),len(data['time'])-1)
        state.qpos[:]=data['q'][i];mujoco.mj_forward(model,state);renderer.update_scene(state,camera=camera)
        frame=renderer.render().copy();phase='HOLD - ADM ON'
        for result in results:
            if result['start_stamp']<=stamp<=result['end_stamp']:phase=result['test']+' | ADM ON'
        cv2.rectangle(frame,(0,0),(960,110),(18,18,18),-1)
        for y,text in [(32,'SUMMIT + UR5e | 9-DOF ROS ACTION'),(64,phase),
                       (96,'t=%.2fs | base x=%.3f m  y=%.3f m  yaw=%.2f deg'%(stamp-data['time'][0],*state.qpos[:2],np.rad2deg(state.qpos[2])))]:
            cv2.putText(frame,text,(18,y),cv2.FONT_HERSHEY_SIMPLEX,.65,(240,240,240),1,cv2.LINE_AA)
        writer.send(frame)
finally:writer.close();renderer.close()
print(root/'mobile_actions.mp4')

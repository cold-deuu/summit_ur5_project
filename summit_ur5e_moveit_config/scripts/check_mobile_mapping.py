#!/usr/bin/env python3
"""Compare imported URDF FK with named MuJoCo bodies at nonzero 9-DOF poses."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

def rotation(axis, angle):
    axis=np.array(axis,dtype=float);axis/=np.linalg.norm(axis)
    x,y,z=axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+np.sin(angle)*K+(1-np.cos(angle))*(K@K)
def fk(root,positions):
    transforms={'world':np.eye(4)};pending=list(root.findall('joint'))
    while pending:
        progressed=False
        for joint in list(pending):
            parent=joint.find('parent').get('link')
            if parent not in transforms:continue
            child=joint.find('child').get('link');T=np.eye(4);o=joint.find('origin')
            if o is not None:
                T[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
                r,p,y=np.fromstring(o.get('rpy','0 0 0'),sep=' ')
                T[:3,:3]=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
            q=positions.get(joint.get('name'),0);A=np.eye(4)
            if joint.get('type')!='fixed':
                axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                if joint.get('type')=='prismatic':A[:3,3]=axis*q
                else:A[:3,:3]=rotation(axis,q)
            transforms[child]=transforms[parent]@T@A;pending.remove(joint);progressed=True
        if not progressed:raise RuntimeError('Disconnected URDF')
    return transforms

def main():
    config=Path(__file__).resolve().parents[1]/'config';mapping=json.loads((config/'mobile_mapping.json').read_text())
    model=mujoco.MjModel.from_xml_path(str(config/'mobile_mujoco_scene.xml'));data=mujoco.MjData(model)
    urdf=ET.parse(config/'mobile_action_control.urdf').getroot()
    pairs={'summit_xl_base_footprint':'base_link','ur5e_base_link':'base'}
    for link in ['shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link']:pairs['ur5e_'+link]=link
    for body in ['ur5e_tool0','graps_link','gripper_base_link']:pairs[body]=body
    names=list(mapping['base'])+list(mapping['arm_joints']);mj_names=list(mapping['base'].values())+list(mapping['arm_joints'].values())
    addresses=[model.jnt_qposadr[model.joint(n).id] for n in mj_names]
    maximum_position=maximum_rotation=0.
    for q in [[0,0,0,0,-1.57,1.57,-1.57,-1.57,0],[.3,-.2,.4,-.8,-1.1,1.7,-1.5,-.9,.6],[-.2,.1,-.6,.5,-1.4,1.2,-1.1,-1.3,-.5]]:
        data.qpos[addresses]=q;mujoco.mj_forward(model,data);transforms=fk(urdf,dict(zip(names,q)))
        for link,body in pairs.items():
            T=transforms[link];b=data.body(body)
            maximum_position=max(maximum_position,float(np.max(np.abs(T[:3,3]-b.xpos))))
            maximum_rotation=max(maximum_rotation,float(np.max(np.abs(T[:3,:3]-b.xmat.reshape(3,3)))))
    result=dict(poses=3,bodies=len(pairs),nq=model.nq,nv=model.nv,nu=model.nu,max_position_error_m=maximum_position,max_rotation_matrix_error=maximum_rotation)
    print(json.dumps(result,indent=2));assert maximum_position<1e-8 and maximum_rotation<1e-8
if __name__=='__main__':main()

#!/usr/bin/env python3
"""Validate existing JointPosture/SE3 actions against the mobile simulation."""
import json
from pathlib import Path
import time
import actionlib
import mujoco
import numpy as np
import rospy
import rospkg
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_matrix
from rci_action_manager.msg import JointPostureAction, JointPostureGoal, SE3Action, SE3Goal


def main():
    rospy.init_node('mobile_action_validation')
    history=[]
    rospy.Subscriber('/mujoco/joint_states',JointState,history.append,queue_size=1000)
    deadline=time.monotonic()+15
    while len(history)<20:
        if time.monotonic()>deadline:raise RuntimeError('Missing simulation states')
        time.sleep(.05)
    model=mujoco.MjModel.from_xml_path(str(Path(rospkg.RosPack().get_path(
        'summit_ur5e_moveit_config'))/'config/mobile_mujoco_scene.xml'))
    data=mujoco.MjData(model)
    def pose(q):
        data.qpos[:]=q;mujoco.mj_forward(model,data)
        wrist=data.body('wrist_3_link')
        return wrist.xpos.copy(),wrist.xmat.reshape(3,3).copy()
    results=[]
    def run(name,kind,goal):
        client=actionlib.SimpleActionClient('/rci_action_manager/'+name,kind)
        assert client.wait_for_server(rospy.Duration(10))
        start=len(history);q0=np.array(history[-1].position[:9])
        client.send_goal(goal)
        assert client.wait_for_result(rospy.Duration(25))
        q=np.array(history[-1].position[:9])
        samples=np.array([s.position[:9] for s in history[start:]])
        assert np.isfinite(samples).all() and client.get_state()==3
        result=dict(start_stamp=history[start].header.stamp.to_sec(),end_stamp=history[-1].header.stamp.to_sec(),test=name,status=client.get_state(),initial=q0.tolist(),final=q.tolist(),
                    base_delta=(q[:3]-q0[:3]).tolist(),
                    max_base_excursion=np.max(np.abs(samples[:,:3]-q0[:3]),axis=0).tolist())
        results.append(result)
        return result
    joint=JointPostureGoal();joint.duration=4
    joint.target_joints.position=[-.35,-1.3,1.6,-1.6,-1.3,.3]
    result=run('jointControl',JointPostureAction,joint)
    result['joint_error_max_rad']=float(np.max(np.abs(np.array(result['final'])[3:]-joint.target_joints.position)))
    assert result['joint_error_max_rad']<.03
    assert max(np.abs(result['base_delta']))<.005
    initial,R=pose(history[-1].position[:9])
    angle=.5;world_rotation=np.array([[np.cos(angle),-np.sin(angle),0],
                                    [np.sin(angle),np.cos(angle),0],[0,0,1]])
    local=np.eye(4);local[:3,:3]=R.T@world_rotation@R
    quat=quaternion_from_matrix(local)
    se3=SE3Goal();se3.duration=5;se3.relative=True;se3.wholebody=True
    se3.target_pose.position.x=.18;se3.target_pose.position.y=.10
    (se3.target_pose.orientation.x,se3.target_pose.orientation.y,
     se3.target_pose.orientation.z,se3.target_pose.orientation.w)=quat
    result=run('se3Control',SE3Action,se3)
    final,final_R=pose(result['final'])
    result['position_error_m']=float(np.linalg.norm(final-initial-np.array([.18,.1,0])))
    result['rotation_error_rad']=float(np.arccos(np.clip((np.trace((world_rotation@R).T@final_R)-1)/2,-1,1)))
    assert result['position_error_m']<.005 and result['rotation_error_rad']<.01
    assert min(np.abs(result['base_delta']))>.001
    out=Path(rospy.get_param('~output','artifacts/moveit_optimal_validation/mobile_actions'))
    out.mkdir(exist_ok=True,parents=True)
    (out/'results.json').write_text(json.dumps(results,indent=2))
    np.savez_compressed(out/'states.npz',time=np.array([s.header.stamp.to_sec() for s in history]),
                        q=np.array([s.position[:9] for s in history]))
    print(json.dumps(results,indent=2),flush=True)

if __name__=='__main__':main()

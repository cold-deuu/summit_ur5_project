#!/usr/bin/env python3
"""Plan only, publish JointTrajectory, and verify the topic controller end to end."""
import argparse
import copy
import json
import os
from pathlib import Path
import threading
import time

import numpy as np
import rospy
import yaml
import genpy.message
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped
from rci_action_manager.msg import MoveitOptimalStatus
from rci_action_manager.srv import SetAdmittanceMode

class Suite:
    def __init__(self,out):
        self.out=Path(out); self.out.mkdir(parents=True,exist_ok=True)
        self.status=None; self.history=[]; self.results=[]
        (self.out/"model_config.json").write_text(json.dumps(dict(
            model_file=rospy.get_param("/mujoco_backend/model_file",""),
            mobile=rospy.get_param("/mujoco_backend/mobile_base",False)),indent=2))
        rospy.Subscriber("/moveit_optimal_controller/status",MoveitOptimalStatus,self.update,queue_size=200)
        self.publisher=rospy.Publisher("/rci_action_manager/moveit_plan",JointTrajectory,queue_size=1)
        self.force=rospy.Publisher("/mujoco/apply_wrench",WrenchStamped,queue_size=1)
        self.mode=rospy.ServiceProxy("/moveit_optimal_controller/set_admittance_mode",SetAdmittanceMode)
        self.wait(lambda s:s.ft_bias_ready and s.command_enabled,20)
        while not self.publisher.get_num_connections(): time.sleep(.05)
    def update(self,s):
        self.status=s; self.history.append(s)
    def wait(self,predicate,timeout=25):
        end=time.monotonic()+timeout
        while time.monotonic()<end and not rospy.is_shutdown():
            if self.status is not None and predicate(self.status): return self.status
            time.sleep(.01)
        s=self.status
        raise RuntimeError("Status timeout: "+str((s.state,s.last_result,s.reason,s.trajectory_id,s.nominal_error_max) if s else None))
    def record(self,name,**values):
        result=dict(test=name,passed=True,**values); self.results.append(result)
        (self.out/"results.json").write_text(json.dumps(self.results,indent=2))
        print(json.dumps(result),flush=True)
    def publish(self,traj):
        before=max(self.status.trajectory_id,self.status.result_trajectory_id)
        self.publisher.publish(traj)
        s=self.wait(lambda s:s.trajectory_id>before or (s.last_result=="REJECTED" and s.result_trajectory_id>before),5)
        if s.last_result=="REJECTED" and s.result_trajectory_id>s.trajectory_id:
            raise RuntimeError("Unexpected reject: "+s.reason)
        return s.trajectory_id
    def success(self,identifier,timeout=35):
        s=self.wait(lambda s:s.trajectory_id==identifier and s.state=="HOLDING" and
                    s.result_trajectory_id==identifier and s.last_result in ("SUCCEEDED","ABORTED"),timeout)
        if s.last_result!="SUCCEEDED": raise RuntimeError("Trajectory aborted: "+s.reason)
        return s
    def execute(self,name,traj,save=False):
        traj.header.stamp=rospy.Time(0)
        if save:
            (self.out/"trajectory.json").write_text(json.dumps(yaml.safe_load(str(traj)),indent=2))
        start=len(self.history)
        identifier=self.publish(traj); s=self.success(identifier)
        samples=[s for s in self.history[start:] if s.trajectory_id==identifier and s.state=="TRACKING"]
        errors=np.array([s.tracking_error for s in samples])
        self.record(name,trajectory_id=identifier,rms_rad=float(np.sqrt(np.mean(errors**2))),
                    max_rad=float(np.max(np.abs(errors))),goal_max_rad=s.nominal_error_max,
                    elapsed_s=s.elapsed)
        return identifier
    def reject(self,name,traj):
        before=max(self.status.result_trajectory_id,self.status.trajectory_id)
        self.publisher.publish(traj)
        s=self.wait(lambda s:s.last_result=="REJECTED" and s.result_trajectory_id>before,5)
        self.record(name,reason=s.reason)
    def push(self,duration,force=6):
        begin=time.monotonic()
        while time.monotonic()-begin<duration:
            msg=WrenchStamped(); msg.header.stamp=rospy.Time.now(); msg.header.frame_id="world"
            msg.wrench.force.x=force; self.force.publish(msg); time.sleep(.02)
        self.force.publish(WrenchStamped())
    def save_history(self):
        if not self.history: return
        history=list(self.history) # Freeze one coherent callback snapshot.
        np.savez_compressed(self.out/"status_history.npz",
            time=np.array([s.header.stamp.to_sec() for s in history]),
            actual=np.array([s.actual for s in history]),
            desired=np.array([s.desired for s in history]),
            command=np.array([s.command for s in history]),
            velocity=np.array([s.command_velocity for s in history]),
            displacement=np.array([s.admittance_displacement for s in history]),
            state=np.array([s.state for s in history]))
    def pose(self,q):
        from moveit_msgs.srv import GetPositionFK,GetPositionFKRequest
        request=GetPositionFKRequest();request.header.frame_id="ur5e_base_link"
        request.fk_link_names=["ur5e_wrist_3_link"]
        request.robot_state.joint_state.name=rospy.get_param("/moveit_optimal_controller/joint_names")
        request.robot_state.joint_state.position=list(q)
        response=rospy.ServiceProxy("/compute_fk",GetPositionFK)(request)
        if response.error_code.val!=1: raise RuntimeError("FK failed")
        p=response.pose_stamped[0].pose.position
        return np.array([p.x,p.y,p.z])
    def compliance(self,mode):
        assert self.mode(mode).success
        time.sleep(1)
        equilibrium=self.pose(self.status.actual)
        self.push(.6)
        pushed=self.pose(self.status.actual)
        release=time.monotonic(); traces=[]
        for _ in range(220):
            traces.append((time.monotonic()-release,self.pose(self.status.actual)))
            time.sleep(.02)
        last=np.mean([p for _,p in traces[-20:]],axis=0)
        excursion=float(np.linalg.norm(pushed-equilibrium))
        assert excursion>.001,("No compliant response",mode,excursion)
        if mode=="spring":
            error=float(np.linalg.norm(last-equilibrium))
            assert error<.003,("Spring did not return",error)
            settle=next((t for i,(t,p) in enumerate(traces)
                         if all(np.linalg.norm(pp-equilibrium)<.003 for _,pp in traces[i:])),None)
            self.record("holding_spring",excursion_m=excursion,return_error_m=error,settling_s=settle)
        else:
            drift=float(np.linalg.norm(traces[-1][1]-traces[50][1]))
            retained=float(np.linalg.norm(last-equilibrium))
            assert retained>.002 and drift<.002,("Hold drift",retained,drift)
            self.record("holding_hold",excursion_m=excursion,retained_displacement_m=retained,drift_m=drift)
    def continuity(self,samples):
        t=np.array([s.header.stamp.to_sec() for s in samples])
        q=np.array([s.command for s in samples]);v=np.array([s.command_velocity for s in samples])
        dt=np.diff(t); good=dt>.001
        speed=np.max(np.abs(np.diff(q,axis=0)[good])/dt[good,None])
        accel=np.max(np.abs(np.diff(v,axis=0)[good])/dt[good,None])
        assert speed<.55 and accel<1.1,("Command discontinuity",speed,accel)
        return dict(max_command_speed_rad_s=float(speed),max_command_acceleration_rad_s2=float(accel))
    def full(self):
        import moveit_commander
        moveit_commander.roscpp_initialize([])
        group=moveit_commander.MoveGroupCommander("manipulator",wait_for_servers=15)
        robot=moveit_commander.RobotCommander()
        group.set_planning_time(5)
        group.set_max_velocity_scaling_factor(.5)
        group.set_max_acceleration_scaling_factor(.5)
        def retime(plan):
            # Recompute derivatives from the planned positions. IPTP endpoint
            # accelerations can be slightly outside their own scaled limits.
            plan=copy.deepcopy(plan)
            for point in plan.joint_trajectory.points:
                point.velocities=[0.0]*len(point.positions)
                point.accelerations=[0.0]*len(point.positions)
            result=group.retime_trajectory(robot.get_current_state(),plan,.5,.5,
                                          algorithm="iterative_spline_parameterization")
            assert len(result.joint_trajectory.points)>=2,"Spline retiming failed"
            return result
        def plan_joint(delta):
            group.set_start_state_to_current_state()
            target=np.array(group.get_current_joint_values())+np.array(delta)
            group.set_joint_value_target(target.tolist())
            result=group.plan()
            assert result[0],"MoveIt joint planning failed"
            timed=retime(result[1])
            return timed.joint_trajectory
        home=np.array([0,-1.57,1.57,-1.57,-1.57,0])
        delta=home-np.array(group.get_current_joint_values())
        if np.max(np.abs(delta))>.01:
            self.execute("return_to_test_home",plan_joint(delta))
        self.execute("joint_plan",plan_joint([-.12,.08,-.06,.04,.03,.05]),save=True)
        group.set_start_state_to_current_state()
        pose=group.get_current_pose().pose;pose.position.x+=.02;pose.position.z+=.015
        group.set_pose_target(pose)
        result=group.plan();group.clear_pose_targets()
        assert result[0],"MoveIt pose planning failed"
        timed=retime(result[1])
        self.execute("pose_plan",timed.joint_trajectory)
        group.set_start_state_to_current_state()
        pose=group.get_current_pose().pose;goal=copy.deepcopy(pose)
        goal.position.x-=.02;goal.position.y+=.01
        cart,fraction=group.compute_cartesian_path([goal],.005)
        assert fraction>.99,("Cartesian fraction",fraction)
        cart=retime(cart)
        self.execute("cartesian_plan",cart.joint_trajectory)
        self.record("cartesian_fraction",fraction=float(fraction))

        start=len(self.history)
        identifier=self.publish(plan_joint([-.25,0,0,0,0,0]))
        self.wait(lambda s:s.trajectory_id==identifier and s.elapsed>.6 and s.state=="TRACKING",10)
        replacement=plan_joint([0,-.08,.06,0,0,0])
        newer=self.publish(replacement);self.success(newer)
        assert newer>identifier
        self.record("replace_while_tracking",old_id=identifier,new_id=newer,
                    **self.continuity(self.history[start:]))
        shuffled=plan_joint([.04,0,0,0,0,0])
        shuffled.joint_names=list(reversed(shuffled.joint_names))
        for point in shuffled.points:
            point.positions=list(reversed(point.positions))
            point.velocities=list(reversed(point.velocities))
            point.accelerations=list(reversed(point.accelerations))
        self.execute("shuffled_joint_names",shuffled)
        good=plan_joint([.04,0,0,0,0,0])
        bad=copy.deepcopy(good)
        bad.points[0].positions=list(bad.points[0].positions)
        bad.points[0].positions[0]+=.2
        self.reject("reject_start_mismatch",bad)
        self.reject("reject_empty",JointTrajectory())
        bad=copy.deepcopy(good);bad.joint_names=list(bad.joint_names)
        bad.joint_names[0]="unknown_joint"
        self.reject("reject_unknown_joint",bad)
        bad=copy.deepcopy(good);bad.points[1].time_from_start=bad.points[0].time_from_start
        self.reject("reject_nonmonotonic_time",bad)
        bad=copy.deepcopy(good);bad.header.stamp=rospy.Time.now()-rospy.Duration(1)
        self.reject("reject_past_stamp",bad)
        future=plan_joint([.03,0,0,0,0,0]);future.header.stamp=rospy.Time.now()+rospy.Duration(1)
        before=np.array(self.status.command)
        identifier=self.publish(future);time.sleep(.4)
        assert np.max(np.abs(np.array(self.status.command)-before))<.001,"Future plan started early"
        self.success(identifier);self.record("future_stamp")

        self.compliance("spring")
        self.compliance("hold")
        start=len(self.history)
        self.push(.25)
        assert self.mode("spring").success
        time.sleep(2)
        assert self.mode("hold").success
        time.sleep(.3)
        self.record("runtime_mode_switch",**self.continuity(self.history[start:]))
        assert self.mode("spring").success
        time.sleep(3)
        traj=plan_joint([-.12,.05,0,0,0,0])
        identifier=self.publish(traj)
        self.wait(lambda s:s.trajectory_id==identifier and s.elapsed>.5,10)
        self.push(.4);self.success(identifier,35)
        assert self.status.nominal_error_max<.015
        self.record("tracking_force_return",goal_max_rad=self.status.nominal_error_max,
                    rms_rad=self.status.tracking_rms,max_rad=self.status.tracking_max)
        self.record("loop_statistics",max_step_ms=self.status.loop_duration_max_ms,
                    missed_deadlines=self.status.missed_deadlines)
        self.save_history()
        # roscpp_shutdown can wait on background Python bindings in older MoveIt;
        # no hardware commands or actions are ever executed by this planner.
    def replay(self,path):
        msg=JointTrajectory()
        genpy.message.fill_message_args(msg,[json.loads(Path(path).read_text())])
        msg.header.stamp=rospy.Time(0)
        self.execute("real_wrapper_mock_replay",msg)
        self.save_history()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",required=True)
    parser.add_argument("--replay",help="Replay saved trajectory against the mock real wrapper, without MoveIt")
    args=parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("test_moveit_optimal",anonymous=True)
    suite=Suite(args.output)
    try:
        if args.replay: suite.replay(args.replay)
        else: suite.full()
    except BaseException as error:
        suite.save_history()
        suite.results.append(dict(test="failure",passed=False,error=repr(error)))
        (suite.out/"results.json").write_text(json.dumps(suite.results,indent=2))
        print("FAILED:",repr(error),flush=True)
        os._exit(1)
    print("ALL TESTS PASSED",flush=True)
    os._exit(0)

if __name__=="__main__": main()

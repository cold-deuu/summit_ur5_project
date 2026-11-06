#!/usr/bin/env python3
"""2 s HOLD -> MoveIt topic tracking -> HOLD, push, release, displaced HOLD."""
import argparse
import copy
import importlib.util
import json
import os
from pathlib import Path
import time

import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import SetBool

spec=importlib.util.spec_from_file_location("topic_suite",Path(__file__).with_name("test_moveit_optimal.py"))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",required=True)
    args=parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("test_moveit_hold_push",anonymous=True)
    suite=module.Suite(args.output)
    events=[]
    def phase(name,force=0):
        events.append(dict(stamp=rospy.Time.now().to_sec(),phase=name,force_N=force))
        (suite.out/"scenario_events.json").write_text(json.dumps(events,indent=2))
        print(name,flush=True)
    enable=rospy.ServiceProxy("/moveit_optimal_controller/set_tracking_admittance",SetBool)
    assert enable(True).success
    assert suite.mode("hold").success
    suite.force.publish(WrenchStamped())
    moveit_commander.roscpp_initialize([])
    group=moveit_commander.MoveGroupCommander("manipulator",wait_for_servers=15)
    robot=moveit_commander.RobotCommander()
    group.set_planning_time(8)
    group.set_max_velocity_scaling_factor(.75)
    group.set_max_acceleration_scaling_factor(.6)
    metadata=dict(admittance_enabled=True,admittance_mode="hold",initial_hold_seconds=2.0,
                  applied_force_world_N=[6,0,0],force_duration_seconds=.8)
    try:
        start_stamp=rospy.Time.now().to_sec()
        initial=np.array(suite.status.actual)
        phase("INITIAL HOLD - 2 SECONDS")
        time.sleep(2.0)
        initial_samples=[s for s in suite.history if s.header.stamp.to_sec()>=start_stamp]
        drift=float(np.max(np.abs(np.array([s.actual for s in initial_samples])-initial)))
        assert drift<.001,("Initial HOLD drift",drift)
        suite.record("initial_hold_2_seconds",max_joint_drift_rad=drift)
        phase("MOVEIT PLANNING")
        group.set_start_state_to_current_state()
        goal=[-1.0,-1.15,1.85,-1.8,-1.0,.8]
        group.set_joint_value_target(goal)
        planned=group.plan()
        assert planned[0],"MoveIt planning failed"
        plan=copy.deepcopy(planned[1])
        for point in plan.joint_trajectory.points:
            point.velocities=[0.0]*6;point.accelerations=[0.0]*6
        timed=group.retime_trajectory(robot.get_current_state(),plan,.75,.6,
                                     algorithm="iterative_spline_parameterization")
        phase("TRACKING - MOVEIT TRAJECTORY")
        suite.execute("moveit_tracking",timed.joint_trajectory,save=True)
        phase("FINAL HOLD - TRAJECTORY COMPLETE")
        time.sleep(1.0)
        equilibrium_q=np.array(suite.status.actual)
        equilibrium=suite.pose(equilibrium_q)
        metadata["nominal_goal_q"]=equilibrium_q.tolist()
        (suite.out/"scenario.json").write_text(json.dumps(metadata,indent=2))
        phase("HOLD - EXTERNAL PUSH 6 N",6)
        suite.push(.8,6)
        phase("FORCE RELEASED - KEEP DISPLACED POSITION")
        time.sleep(1.0) # mass-damper velocity decays; no spring restoration
        settled=suite.pose(suite.status.actual)
        positions=[]
        for _ in range(60):
            positions.append(suite.pose(suite.status.actual))
            time.sleep(.05)
        final=np.mean(positions[-10:],axis=0)
        displacement=float(np.linalg.norm(final-equilibrium))
        drift=float(np.linalg.norm(final-settled))
        assert displacement>.01,("No retained displacement",displacement)
        assert drift<.002,("Displaced HOLD drift",drift)
        suite.record("hold_after_push",retained_ee_displacement_m=displacement,
                     post_settle_drift_m=drift,force_N=6,force_duration_s=.8)
        phase("DISPLACED HOLD - VERIFIED")
        time.sleep(1.0)
        stop_stamp=rospy.Time.now().to_sec()
        history=[s for s in suite.history if start_stamp<=s.header.stamp.to_sec()<=stop_stamp]
        np.savez_compressed(suite.out/"status_history.npz",
            time=np.array([s.header.stamp.to_sec() for s in history]),
            actual=np.array([s.actual for s in history]),desired=np.array([s.desired for s in history]),
            command=np.array([s.command for s in history]),velocity=np.array([s.command_velocity for s in history]),
            displacement=np.array([s.admittance_displacement for s in history]),
            state=np.array([s.state for s in history]))
        (suite.out/"scenario.json").write_text(json.dumps(metadata,indent=2))
        print("HOLD -> TRACK -> PUSH -> DISPLACED HOLD PASSED",flush=True)
    except BaseException as exc:
        suite.save_history()
        suite.results.append(dict(test="failure",passed=False,error=repr(exc)))
        (suite.out/"results.json").write_text(json.dumps(suite.results,indent=2))
        print("FAILED:",repr(exc),flush=True)
        os._exit(1)
    os._exit(0)

if __name__=="__main__":main()

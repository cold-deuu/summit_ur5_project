#!/usr/bin/env python3
"""Large joint-space MoveIt plans -> the same topic controller; no external force."""
import argparse
import copy
import importlib.util
import os
from pathlib import Path
import time
import moveit_commander
import numpy as np
import rospy
from std_srvs.srv import SetBool
from geometry_msgs.msg import WrenchStamped

spec=importlib.util.spec_from_file_location("topic_suite",Path(__file__).with_name("test_moveit_optimal.py"))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",required=True)
    args=parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("test_moveit_long_tracking",anonymous=True)
    suite=module.Suite(args.output)
    enable=rospy.ServiceProxy("/moveit_optimal_controller/set_tracking_admittance",SetBool)
    assert enable(True).success
    suite.mode("spring")
    suite.force.publish(WrenchStamped()) # Explicitly clear any earlier simulated push.
    time.sleep(.5)
    moveit_commander.roscpp_initialize([])
    group=moveit_commander.MoveGroupCommander("manipulator",wait_for_servers=15)
    robot=moveit_commander.RobotCommander()
    group.set_planning_time(8)
    group.set_max_velocity_scaling_factor(.75)
    group.set_max_acceleration_scaling_factor(.6)
    goals=[
        ("wide_left",[-1.0,-1.15,1.85,-1.8,-1.0,.8]),
        ("wide_right",[1.0,-1.8,1.1,-1.05,-1.95,-.8]),
        ("reach_and_turn",[-.55,-.95,1.25,-1.95,-1.1,.6]),
        ("return_home",[0,-1.57,1.57,-1.57,-1.57,0]),
    ]
    try:
        for name,goal in goals:
            start=np.array(suite.status.actual)
            start_position=suite.pose(start)
            group.set_start_state_to_current_state()
            group.set_joint_value_target(goal)
            result=group.plan()
            assert result[0],"MoveIt planning failed: "+name
            plan=copy.deepcopy(result[1])
            for point in plan.joint_trajectory.points:
                point.velocities=[0.0]*6;point.accelerations=[0.0]*6
            timed=group.retime_trajectory(robot.get_current_state(),plan,.75,.6,
                                         algorithm="iterative_spline_parameterization")
            assert len(timed.joint_trajectory.points)>=2,"Spline retiming failed"
            index=len(suite.history)
            suite.execute(name,timed.joint_trajectory)
            samples=suite.history[index:]
            # Use measured FK along the path, not only planned endpoint distance.
            positions=np.array([suite.pose(s.actual) for s in samples[::10]])
            path=float(np.sum(np.linalg.norm(np.diff(positions,axis=0),axis=1)))
            endpoint=float(np.linalg.norm(suite.pose(suite.status.actual)-start_position))
            suite.results[-1].update(max_joint_travel_rad=float(np.max(np.abs(np.array(goal)-start))),
                measured_ee_path_m=path,ee_endpoint_displacement_m=endpoint,
                no_external_force=True,tracking_admittance=True)
            (suite.out/"results.json").write_text(__import__("json").dumps(suite.results,indent=2))
            print(name,"measured EE path",round(path,3),"m",flush=True)
            time.sleep(.5)
        suite.save_history()
        print("ALL LONG-TRACKING TESTS PASSED",flush=True)
    except BaseException as exc:
        suite.save_history()
        suite.results.append(dict(test="failure",passed=False,error=repr(exc)))
        (suite.out/"results.json").write_text(__import__("json").dumps(suite.results,indent=2))
        print("FAILED:",repr(exc),flush=True)
        os._exit(1)
    os._exit(0)

if __name__=="__main__":main()

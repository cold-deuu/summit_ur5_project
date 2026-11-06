#!/usr/bin/env python3
"""Inspect by default; --execute switches controllers at measured standstill.
No joint goal, URScript, program Play or protective-stop recovery is sent.
"""
import argparse,json
import rospy,rosgraph
from std_msgs.msg import Bool
from ur_dashboard_msgs.msg import SafetyMode
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers,LoadController,SwitchController,SwitchControllerRequest
from first_joint_check import ordered_state,NAMES

CONTROLLER='hqp_joint_position_controller'
def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true')
    args=parser.parse_args(rospy.myargv()[1:]);rospy.init_node('switch_ur_position_controller',anonymous=True)
    rospy.wait_for_service('/controller_manager/list_controllers',timeout=5)
    listing=rospy.ServiceProxy('/controller_manager/list_controllers',ListControllers)
    values=listing().controller
    def claims(c):return any(set(r.resources)&set(NAMES) for r in c.claimed_resources)
    stop=[c.name for c in values if c.name!=CONTROLLER and c.state=='running' and claims(c)]
    print(json.dumps(dict(mode='SWITCH' if args.execute else 'INSPECT_ONLY',target=CONTROLLER,
        currently_running=[c.name for c in values if c.state=='running'],would_stop=stop),indent=2),flush=True)
    if not args.execute:return
    # A running publisher could replace the seed immediately on activation.
    publications=rosgraph.Master(rospy.get_name()).getSystemState()[0]
    for topic,nodes in publications:
        if topic in ['/hqp_joint_position_controller/command','/scaled_pos_joint_traj_controller/command'] and nodes:
            raise RuntimeError('Stop command publishers before switching: '+str(nodes))
    state=rospy.wait_for_message('/joint_states',JointState,timeout=5)
    ordered_state(state,rospy.Time.now().to_sec())
    program=rospy.wait_for_message('/ur_hardware_interface/robot_program_running',Bool,timeout=5)
    safety=rospy.wait_for_message('/ur_hardware_interface/safety_mode',SafetyMode,timeout=5)
    if not program.data or safety.mode!=SafetyMode.NORMAL:raise RuntimeError('External Control must run with NORMAL safety state')
    existing=next((c for c in values if c.name==CONTROLLER),None)
    if existing is not None:
        if existing.type!='position_controllers/JointGroupPositionController' or rospy.get_param('/'+CONTROLLER+'/joints',[])!=NAMES:
            raise RuntimeError('Loaded controller type/order differs; unload the stopped controller before reconfiguring')
    else:
        rospy.set_param('/'+CONTROLLER+'/type','position_controllers/JointGroupPositionController')
        rospy.set_param('/'+CONTROLLER+'/joints',NAMES)
    if existing is None:
        rospy.wait_for_service('/controller_manager/load_controller',timeout=5)
        if not rospy.ServiceProxy('/controller_manager/load_controller',LoadController)(CONTROLLER).ok:
            raise RuntimeError('Cannot load direct position controller')
    # Recheck standstill after service waits. Starting() seeds measured positions.
    ordered_state(rospy.wait_for_message('/joint_states',JointState,timeout=5),rospy.Time.now().to_sec())
    request=SwitchControllerRequest();request.start_controllers=[CONTROLLER];request.stop_controllers=stop
    if any(c.name==CONTROLLER and c.state=='running' for c in values):request.start_controllers=[]
    request.strictness=SwitchControllerRequest.STRICT;request.start_asap=False;request.timeout=5.0
    rospy.wait_for_service('/controller_manager/switch_controller',timeout=5)
    if not rospy.ServiceProxy('/controller_manager/switch_controller',SwitchController)(request).ok:
        raise RuntimeError('Controller switch failed')
    updated=listing().controller
    if not any(c.name==CONTROLLER and c.state=='running' for c in updated):raise RuntimeError('Direct controller is not running')
    if any(c.name!=CONTROLLER and c.state=='running' and claims(c) for c in updated):raise RuntimeError('Conflicting motion controller remains')
    print('Direct position controller running; no movement goal was sent.',flush=True)

if __name__=='__main__':main()

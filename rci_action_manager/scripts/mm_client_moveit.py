from __future__ import print_function
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory
import rci_action_manager.msg

class MoveitClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('rci_action_manager/moveitControl', rci_action_manager.msg.MoveitAction)
        rospy.loginfo("Waiting for action server 'rci_action_manager/moveitControl'...")
        self.client.wait_for_server()
        rospy.loginfo("Action server connected.")

        self.target_trajectory = None
        self.new_trajectory_received = False


        rospy.Subscriber("/rci_action_manager/moveit_plan", JointTrajectory, self.moveit_callback)

    def moveit_callback(self, msg):
        rospy.loginfo("New trajectory received from '/rci_action_manager/moveit_plan' topic.")
        self.target_trajectory = msg
        self.new_trajectory_received = True
        print("HERE")

    def execute_moveit_goal(self):
        if not self.target_trajectory:
            rospy.logwarn("No trajectory to execute.")
            return

        goal = rci_action_manager.msg.MoveitActionGoal
        goal.target_traj = self.target_trajectory
        
        # Debug Goal Start
        print("="*50)
        print("Sending the following goal to the action server:")
        print(goal.target_traj)
        print("="*50)
        # Debug Goal End

        rospy.loginfo("Sending goal to action server...")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_result():
            rospy.loginfo("Action Succeeded!")
        else:
            rospy.logerr("Action Failed!")


if __name__ == '__main__':
    try:
        rospy.init_node('moveit_client')        
        moveit_client = MoveitClient()
        rate = rospy.Rate(1000) 

        while not rospy.is_shutdown():
            if moveit_client.new_trajectory_received:                
                moveit_client.new_trajectory_received = False                
                moveit_client.execute_moveit_goal()
                rospy.loginfo("Waiting for the next trajectory...")

            rate.sleep()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
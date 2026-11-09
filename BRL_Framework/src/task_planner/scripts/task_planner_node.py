#!/usr/bin/env python3
import rospy
from task_planner.task_planner import TaskPlannerNode

if __name__ == '__main__':
    try:
        TaskPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#include "rci_action_manager/controller/moveit_topic_node.hpp"
#include <std_msgs/Float64MultiArray.h>
int main(int argc,char **argv) {
    ros::init(argc,argv,"moveit_optimal_controller");
    try {
        moveit_optimal::TopicNode node;
        auto state=node.nh.subscribe<sensor_msgs::JointState>(node.state_topic,1,&moveit_optimal::TopicNode::receiveState,&node);
        auto wrench=node.nh.subscribe<std_msgs::Float64MultiArray>(node.wrench_topic,1,
            [&node](const std_msgs::Float64MultiArray::ConstPtr &msg) {
                if(msg->data.size()!=6) return;
                node.receiveWrench(Eigen::Map<const moveit_optimal::Vec>(msg->data.data()),ros::Time::now().toSec());
            });
        auto pub=node.nh.advertise<std_msgs::Float64MultiArray>(node.command_topic,1);
        node.run([&](const moveit_optimal::Snapshot &s) {
            std_msgs::Float64MultiArray msg; msg.data.assign(s.command.begin(),s.command.end()); pub.publish(msg);
        });
    } catch(const std::exception &e) { ROS_FATAL("%s",e.what()); return 1; }
}

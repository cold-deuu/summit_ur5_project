#include "rci_action_manager/controller/moveit_topic_node.hpp"
#include "rci_action_manager/controller/position_command_output.hpp"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
int main(int argc,char **argv) {
    ros::init(argc,argv,"moveit_optimal_controller");
    try {
        moveit_optimal::TopicNode node;
        if(node.parameters().require_driver_ready) node.receiveDriverProgram(false);
        auto program=node.nh.subscribe<std_msgs::Bool>("/ur_hardware_interface/robot_program_running",1,
            [&node](const std_msgs::Bool::ConstPtr &msg) { node.receiveDriverProgram(msg->data); });
        auto scaling=node.nh.subscribe<std_msgs::Float64>("/speed_scaling_factor",1,
            [&node](const std_msgs::Float64::ConstPtr &msg) { node.receiveDriverScaling(msg->data,ros::Time::now().toSec()); });
        auto state=node.nh.subscribe<sensor_msgs::JointState>(node.state_topic,1,&moveit_optimal::TopicNode::receiveState,&node);
        auto wrench=node.nh.subscribe<geometry_msgs::WrenchStamped>(node.wrench_topic,1,
            [&node](const geometry_msgs::WrenchStamped::ConstPtr &msg) {
                moveit_optimal::Vec value;
                value<<msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z,
                    msg->wrench.torque.x,msg->wrench.torque.y,msg->wrench.torque.z;
                node.receiveWrench(value,msg->header.stamp.isZero()?ros::Time::now().toSec():msg->header.stamp.toSec(),msg->header.frame_id);
            });
        bool enabled;node.private_nh.param("command_enabled",enabled,false);
        const auto &names=node.parameters().joint_names;
        rci_hardware::PositionCommandOutput output(node.nh,node.private_nh,
            std::vector<std::string>(names.begin(),names.end()),enabled,node.command_topic);
        node.run([&](const moveit_optimal::Snapshot &s) { output.publish(s.command.data()); });
    } catch(const std::exception &e) { ROS_FATAL("%s",e.what()); return 1; }
}

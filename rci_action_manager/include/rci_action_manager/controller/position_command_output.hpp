#pragma once
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_manager_msgs/ListControllers.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>

namespace rci_hardware {
// Non-RT transport shared by the legacy action and topic-controller entry points.
// Direct mode publishes positions in configured controller joint order. No spline,
// extrapolation, controller switching, or hardware program startup occurs here.
class PositionCommandOutput {
public:
    PositionCommandOutput(ros::NodeHandle &nh,ros::NodeHandle &params,
                          const std::vector<std::string> &names,bool enabled,
                          const std::string &default_topic="/scaled_pos_joint_traj_controller/command")
        : names_(names),enabled_(enabled) {
        params.param<std::string>("command_interface",interface_,"trajectory");
        params.param<std::string>("command_topic",topic_,default_topic);
        double rate;params.param("rate",rate,100.0);params.param("command_horizon",horizon_,1/rate);
        if(names_.size()!=6 || std::set<std::string>(names_.begin(),names_.end()).size()!=6 ||
           !(rate>0) || !(horizon_>0)) throw std::invalid_argument("Invalid position output configuration");
        if(interface_!="direct_position" && interface_!="trajectory")
            throw std::invalid_argument("command_interface must be direct_position or trajectory");
        bool require_trajectory_controller=false;
        params.param("require_trajectory_controller",require_trajectory_controller,false);
        if(interface_=="trajectory" && enabled_ && require_trajectory_controller) {
            std::string service,controller,joints_parameter;
            params.param<std::string>("controller_manager_service",service,"/controller_manager/list_controllers");
            params.getParam("trajectory_controller",controller);
            params.getParam("controller_joints_parameter",joints_parameter);
            auto client=nh.serviceClient<controller_manager_msgs::ListControllers>(service);
            if(!client.waitForExistence(ros::Duration(2)))throw std::runtime_error("Field controller manager unavailable");
            controller_manager_msgs::ListControllers request;
            if(!client.call(request))throw std::runtime_error("Cannot inspect field controllers");
            bool ready=false;
            for(const auto &value:request.response.controller) {
                if(value.name==controller && value.state=="running" &&
                   (value.type=="position_controllers/ScaledJointTrajectoryController" ||
                    value.type=="position_controllers/JointTrajectoryController"))ready=true;
                if(value.name==controller || value.state!="running")continue;
                for(const auto &resources:value.claimed_resources)
                    for(const auto &joint:resources.resources)
                        if(std::find(names_.begin(),names_.end(),joint)!=names_.end())
                            throw std::runtime_error("Conflicting field motion controller: "+value.name);
            }
            std::vector<std::string> configured;
            if(!ready || !nh.getParam(joints_parameter,configured) || configured.size()!=names_.size() ||
               std::set<std::string>(configured.begin(),configured.end())!=std::set<std::string>(names_.begin(),names_.end()))
                throw std::runtime_error("Field trajectory controller is not running or joint names differ");
        }
        if(interface_=="direct_position") {
            bool require_controller;params.param("require_position_controller",require_controller,true);
            if(enabled_ && require_controller) {
                std::string controller;params.param<std::string>("position_controller",controller,"hqp_joint_position_controller");
                auto client=nh.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
                if(!client.waitForExistence(ros::Duration(2))) throw std::runtime_error("Controller manager unavailable");
                controller_manager_msgs::ListControllers request;
                if(!client.call(request)) throw std::runtime_error("Cannot inspect hardware controllers");
                bool ready=false;
                for(const auto &value:request.response.controller)
                    if(value.name==controller && value.state=="running" &&
                       value.type=="position_controllers/JointGroupPositionController") ready=true;
                for(const auto &value:request.response.controller) {
                    if(value.name==controller || value.state!="running") continue;
                    for(const auto &resources:value.claimed_resources)
                        for(const auto &joint:resources.resources)
                            if(std::find(names_.begin(),names_.end(),joint)!=names_.end())
                                throw std::runtime_error("Another active controller claims an arm joint: "+value.name);
                }
                std::vector<std::string> order;
                if(!ready || !nh.getParam("/"+controller+"/joints",order) || order!=names_)
                    throw std::runtime_error("Direct position controller is not running or joint order differs; switch controllers first");
            }
            publisher_=nh.advertise<std_msgs::Float64MultiArray>(topic_,1);
        } else publisher_=nh.advertise<trajectory_msgs::JointTrajectory>(topic_,1);
        if(enabled_) {
            // Prevent the two local HQP nodes from commanding the same topic.
            uint64_t hash=1469598103934665603ULL;
            for(unsigned char ch:nh.resolveName(topic_)) { hash^=ch;hash*=1099511628211ULL; }
            const std::string path="/tmp/rci_position_owner_"+std::to_string(hash)+".lock";
            lock_=::open(path.c_str(),O_CREAT|O_RDWR,0600);
            if(lock_<0) throw std::runtime_error("Cannot open command ownership lock");
            if(::flock(lock_,LOCK_EX|LOCK_NB)!=0) {
                ::close(lock_);lock_=-1;
                throw std::runtime_error("Another local HQP node owns this command topic; stop it before switching nodes");
            }
        }
        ROS_INFO_STREAM("Position output: "<<interface_<<", topic="<<topic_<<", enabled="<<enabled_);
    }
    ~PositionCommandOutput() { if(lock_>=0) { ::flock(lock_,LOCK_UN);::close(lock_); } }
    PositionCommandOutput(const PositionCommandOutput&)=delete;
    PositionCommandOutput& operator=(const PositionCommandOutput&)=delete;
    void publish(const double *positions) {
        if(!enabled_) return;
        for(int i=0;i<6;++i) if(!std::isfinite(positions[i])) throw std::runtime_error("Nonfinite position command");
        if(interface_=="direct_position") {
            std_msgs::Float64MultiArray msg;msg.data.assign(positions,positions+6);publisher_.publish(msg);
        } else {
            trajectory_msgs::JointTrajectory msg;msg.header.stamp=ros::Time(0);msg.joint_names=names_;
            trajectory_msgs::JointTrajectoryPoint point;point.positions.assign(positions,positions+6);
            point.time_from_start=ros::Duration(horizon_);msg.points.push_back(point);publisher_.publish(msg);
        }
    }
private:
    std::vector<std::string> names_;
    bool enabled_;
    std::string interface_,topic_;
    ros::Publisher publisher_;
    double horizon_=.01;
    int lock_=-1;
};
}

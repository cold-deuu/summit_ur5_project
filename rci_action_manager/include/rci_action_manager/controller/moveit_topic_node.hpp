#pragma once
#include "rci_action_manager/controller/moveit_optimal_controller.hpp"
#include <ros/ros.h>
#include <rci_action_manager/MoveitOptimalStatus.h>
#include <rci_action_manager/ControlTelemetry.h>
#include <rci_action_manager/SetAdmittanceMode.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <functional>
#include <chrono>
#include <thread>
#include <mutex>

namespace moveit_optimal {
class TopicNode {
public:
    TopicNode();
    ~TopicNode();
    void run(const std::function<void(const Snapshot&)> &publish_command);
    void receiveState(const sensor_msgs::JointState::ConstPtr&);
    void receiveWrench(const Vec&,double stamp,const std::string &frame="");
    const Parameters &parameters() const { return params_; }
    void receiveDriverProgram(bool running) { controller_->driver_running.store(running); }
    void receiveDriverScaling(double scale,double stamp) {
        if(std::isfinite(scale) && scale>=0 && scale<=1) { controller_->driver_scale.store(scale);controller_->driver_scaling_stamp.store(stamp); }
    }
    ros::NodeHandle nh,private_nh;
    std::string state_topic,command_topic,wrench_topic,wrench_frame;
private:
    Parameters loadParameters();
    Parameters params_;
    std::unique_ptr<Controller> controller_;
    ros::Subscriber trajectory_sub_,servo_sub_;
    ros::Publisher status_pub_,telemetry_pub_,command_timing_pub_;
    std::chrono::steady_clock::time_point last_command_wall_;
    bool have_command_time_=false;
    std::string telemetry_frame_;
    std::atomic<double> measured_velocity_{1e9};
    ros::ServiceServer mode_service_,bias_service_,enabled_service_;
    ros::WallTimer output_timer_;
    std::thread control_thread_;
    std::atomic<bool> running_{false};
    std::function<void(const Snapshot&)> publish_command_;
    bool command_enabled_=false,bias_enabled_=true,calibrating_=true;
    int bias_samples_=100,bias_count_=0;
    Vec bias_=Vec::Zero(),bias_sum_=Vec::Zero();
    std::mutex bias_mutex_;
    std::atomic<uint64_t> next_id_{0}; // trajectory + servo callbacks (AsyncSpinner(2))
    uint64_t last_command_sequence_=0;
    void receiveTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr&);
    // [BRL_lab] ~servo_enabled: PoseStamped targets for servo_frame in ~servo_reference_frame
    void receiveServo(const geometry_msgs::PoseStamped::ConstPtr&);
    std::string servo_reference_frame_,model_root_;
    double last_servo_receive_=-1;
    uint64_t servo_stream_id_=0;
    void publish(const ros::WallTimerEvent&);
    bool setMode(rci_action_manager::SetAdmittanceMode::Request&,rci_action_manager::SetAdmittanceMode::Response&);
    bool zeroBias(std_srvs::Trigger::Request&,std_srvs::Trigger::Response&);
    bool setEnabled(std_srvs::SetBool::Request&,std_srvs::SetBool::Response&);
    bool snapshot(Snapshot &s);
};
}

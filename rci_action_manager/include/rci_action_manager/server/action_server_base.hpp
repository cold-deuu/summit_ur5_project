#pragma once

#include <rci_action_manager/controller/controller.hpp>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>
#include <map>

#define DEBUG_FILE(text) \
if(debug_file_.is_open()) \
{ \
  debug_file_ << text << std::endl;\
}

class ActionServerBase
{
  protected:
    int feedback_header_stamp_ {0}; 

    std::string action_name_;
    ros::Time start_time_;
    ros::Time trajectory_time_, last_compute_;
    uint64_t generation_=0;
    ros::Time controlTime(ros::Time now) {
      const double dt=std::max(0.0,std::min(.1,(now-last_compute_).toSec()));
      trajectory_time_+=ros::Duration(dt*mu_->speed_scaling());
      last_compute_=now;
      return trajectory_time_;
    }
    ros::NodeHandle & nh_; 
    bool control_running_ {false}; 
    bool mode_change_ {false};
    
    std::ofstream debug_file_;
    std::shared_ptr<RobotController::SummitUR5Wrapper> mu_;
       
    ActionServerBase(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SummitUR5Wrapper> &mu) :
    action_name_(name), nh_(nh), mu_(mu)  {}

    virtual void goalCallback() = 0;
    virtual void preemptCallback() = 0;

  public:
    bool isrunning(){
        return control_running_;
    }
    virtual bool compute(ros::Time time) = 0;
    virtual void signalAbort(bool is_aborted)
    {
      setAborted();
    }

    
  protected:
    int iter_per_print_ {10};
    int print_count_ {0};
    
    virtual void setSucceeded() {};
    virtual void setAborted() {};
};

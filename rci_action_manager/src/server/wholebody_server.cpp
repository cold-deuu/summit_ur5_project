#include <rci_action_manager/server/wholebody_server.hpp>
#include <pinocchio/fwd.hpp>

using namespace pinocchio;

SE3ActionServer::SE3ActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SummitUR5Wrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
    as_.registerGoalCallback(boost::bind(&SE3ActionServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&SE3ActionServer::preemptCallback, this));
    as_.start();
}

void SE3ActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    Eigen::Vector3d pos(goal_->target_pose.position.x, goal_->target_pose.position.y, goal_->target_pose.position.z);
    Eigen::Quaterniond quat(goal_->target_pose.orientation.w, goal_->target_pose.orientation.x, goal_->target_pose.orientation.y, goal_->target_pose.orientation.z);
    if (!mu_->control_ready() || !std::isfinite(goal_->duration) || goal_->duration<=0 ||
        !pos.allFinite() || !quat.coeffs().allFinite() || std::abs(quat.norm()-1)>.001) { setAborted(); return; }
    quat.normalize();
    bool rel = goal_->relative;

    SE3 oMi_ref(quat.toRotationMatrix(), pos);
    // mu_ -> get_se3_task(oMi_ref);
    dur_ = ros::Duration(goal_->duration); 
    start_time_ = ros::Time::now();
    mu_->initControl(oMi_ref,dur_, rel);
    trajectory_time_=last_compute_=start_time_;
    generation_=mu_->generation();
    control_running_ = true;  
}

void SE3ActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  result_.is_completed=false;
  as_.setPreempted(result_);
  control_running_ = false;
}

bool SE3ActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if (!mu_->control_ready() || generation_!=mu_->generation()) { setAborted(); return false; }
  ctime=controlTime(ctime);
  mu_->Control(ctime);
  
  if (ctime.toSec() - start_time_.toSec() > goal_->duration+5.0 && mu_->se3_settled(.015,.03)){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec()  > goal_->duration+ 6.0){
    setAborted();
    return false;
  }

  return false;
}


void SE3ActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void SE3ActionServer::setSucceeded()
{
  result_.is_completed=true;
  as_.setSucceeded(result_);
  
  control_running_ = false;
}
void SE3ActionServer::setAborted()
{
  result_.is_completed=false;
  as_.setAborted(result_);
  control_running_ = false;
}
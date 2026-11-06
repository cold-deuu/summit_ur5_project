#include <rci_action_manager/server/joint_posture_server.hpp>
#include <pinocchio/fwd.hpp>

using namespace pinocchio;

JointPostureActionServer::JointPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SummitUR5Wrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
    as_.registerGoalCallback(boost::bind(&JointPostureActionServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&JointPostureActionServer::preemptCallback, this));
    ros::NodeHandle params("~"); params.param("joint_goal_tolerance",goal_tolerance_,.015);
    as_.start();
}
// Ver. Original
void JointPostureActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    abort_reason_="controller not ready or invalid joint goal";
    if (!mu_->control_ready() || !std::isfinite(goal_->duration) || goal_->duration<=0 ||
        goal_->target_joints.position.size()!=6) { setAborted(); return; }
    Eigen::VectorXd target_arm_posture(6);
    const auto &model=mu_->model(); const int offset=model.nq-6;
    const auto &names=goal_->target_joints.name;
    if (!names.empty() && names.size()!=6) { setAborted(); return; }
    for (int i=0;i<6;++i) {
      int source=i;
      if (!names.empty()) {
        std::string name;
        for (pinocchio::JointIndex j=1;j<model.joints.size();++j)
          if (model.joints[j].idx_q()==offset+i) name=model.names[j];
        if (std::count(names.begin(),names.end(),name)!=1) { setAborted(); return; }
        source=std::distance(names.begin(),std::find(names.begin(),names.end(),name));
      }
      const double q=goal_->target_joints.position[source];
      if (!std::isfinite(q) || q<model.lowerPositionLimit(offset+i) || q>model.upperPositionLimit(offset+i)) {
        setAborted(); return;
      }
      target_arm_posture(i)=q;
    }

    // Cubic rest-to-rest interpolation peaks at 1.5 * abs(delta) / T.
    // Reject impossible timing before initializing HQP or publishing motion.
    const double minimum_duration=(1.5*(target_arm_posture-mu_->state().q.tail(6)).cwiseAbs()
        .cwiseQuotient(mu_->joint_velocity_limits())).maxCoeff();
    if (goal_->duration+1e-6<minimum_duration) {
        abort_reason_="duration infeasible for joint velocity limit; minimum trajectory time="+
            std::to_string(minimum_duration)+" s (before driver scaling/lag)";
        setAborted(); return;
    }
    dur_ = ros::Duration(goal_->duration); 
    start_time_ = ros::Time::now();

    mu_->initJointPosture(target_arm_posture, dur_);
    trajectory_time_=last_compute_=start_time_;
    generation_=mu_->generation();
    control_running_ = true;  
}

// void JointPostureActionServer::goalCallback()
// {
//     feedback_header_stamp_ = 0;
//     goal_ = as_.acceptNewGoal();
//     Eigen::VectorXd target_arm_posture(6);

//     for (int i=0; i<6; i++)
//       target_arm_posture(i) = goal_->target_joints.position[i];

//     dur_ = ros::Duration(goal_->duration); 
//     start_time_ = ros::Time::now();

//     mu_->initJointPosture(target_arm_posture, dur_);
//     control_running_ = true;  
// }


void JointPostureActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  result_.is_completed=false;
  as_.setPreempted(result_);
  control_running_ = false;
}

bool JointPostureActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if (!mu_->control_ready() || generation_!=mu_->generation()) {
    abort_reason_=mu_->control_ready()?"superseded by another action":"controller input/driver not ready";
    setAborted(); return false;
  }
  ctime=controlTime(ctime);
  mu_->jointPosture(ctime);

  
  if (ctime.toSec() - start_time_.toSec() > goal_->duration + 2.0 && mu_->joint_settled(goal_tolerance_)){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec()  > goal_->duration+ 3.0){
    abort_reason_="joint goal tolerance not reached before settling timeout";
    setAborted();
    return false;
  }

  return false;
}


void JointPostureActionServer::signalAbort(bool is_aborted)
{
  abort_reason_="real interface stopped action: input stale, driver inactive or command rejected";
  setAborted();  
}

void JointPostureActionServer::setSucceeded()
{
  result_.is_completed=true;
  as_.setSucceeded(result_);
  
  control_running_ = false;
}
void JointPostureActionServer::setAborted()
{
  result_.is_completed=false;
  ROS_WARN_STREAM("[" << action_name_ << "] " << abort_reason_);
  as_.setAborted(result_,abort_reason_);
  control_running_ = false;
}
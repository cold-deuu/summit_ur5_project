#include <rci_action_manager/server/moveit_server.hpp>
#include <pinocchio/fwd.hpp>

using namespace pinocchio;

MoveitActionServer::MoveitActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SummitUR5Wrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
    as_.registerGoalCallback(boost::bind(&MoveitActionServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&MoveitActionServer::preemptCallback, this));
    as_.start();
}
// Ver. Original
void MoveitActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    trajectory_msgs::JointTrajectory joint_traj;
    
    
    ros::Duration previous_time_from_start(0.0);
    for (const auto& point : goal_->target_traj.points)
    {
      std::cout<<"[MoveitServer] Target Appends!"<<std::endl;
      
      // Debug
      // 이전 지점과의 시간 간격 계산 (ros::Duration 객체 간의 뺄셈)
      ros::Duration interval = point.time_from_start - previous_time_from_start;

      // ROS_INFO를 사용하여 포맷에 맞춰 정보 출력
      ROS_INFO(" Time from start: %.4f s, Interval: %.4f s",
                point.time_from_start.toSec(), // toSec()으로 double 타입 초 단위 변환
                interval.toSec());

      // 다음 계산을 위해 현재 시간을 '이전 시간' 변수에 저장
      previous_time_from_start = point.time_from_start;


      joint_traj.points.push_back(point);
    }
    // OR
    joint_traj = goal_->target_traj;
    mu_->initMoveitPlanner(joint_traj);

    control_running_ = true;  
}


void MoveitActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool MoveitActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  mu_->ControlMoveitPlanner();
  bool is_succed_ = mu_->moveit_succed_;
  bool is_abort_ = mu_->moveit_abort_;

  if (is_succed_){
    setSucceeded();
    return true;
  }

  if (is_abort_){
    setAborted();
    return false;
  }

  return false;
}


void MoveitActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void MoveitActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  
  control_running_ = false;
}
void MoveitActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}
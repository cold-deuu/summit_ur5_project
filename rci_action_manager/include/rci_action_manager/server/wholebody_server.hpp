#pragma once

#include <rci_action_manager/server/action_server_base.hpp>
#include <rci_action_manager/SE3Action.h>
#include "tf/transform_datatypes.h"

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class SE3ActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<rci_action_manager::SE3Action> as_;

  rci_action_manager::SE3Feedback feedback_;
  rci_action_manager::SE3Result result_;
  rci_action_manager::SE3GoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  SE3ActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SummitUR5Wrapper>  &mu);
  ros::Duration dur_;

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};
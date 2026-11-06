#pragma once

#include <rci_action_manager/server/action_server_base.hpp>
#include <rci_action_manager/MoveitAction.h>
#include "tf/transform_datatypes.h"

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class MoveitActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<rci_action_manager::MoveitAction> as_;

  rci_action_manager::MoveitFeedback feedback_;
  rci_action_manager::MoveitResult result_;
  rci_action_manager::MoveitGoalConstPtr goal_;


  void goalCallback() override;
  void preemptCallback() override;

public:
  MoveitActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::SummitUR5Wrapper>  &mu);
  ros::Duration dur_;

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};
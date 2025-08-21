// Standard
#include <string>
#include <vector>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <random>
#include <ctime>


// FCL
#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model-inl.h>
#include <fcl/math/bv/OBBRSS.h>
#include <fcl/math/constants.h>
#include <fcl/narrowphase/collision_object.h>

// Eigen
#include <Eigen/QR>    
#include <Eigen/Core>
#include <Eigen/Dense>

// Pinocchio
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "pinocchio/fwd.hpp"
#include "pinocchio/algorithm/compute-all-terms.hxx"
#include <pinocchio/algorithm/frames.hpp>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// ROS : Message
#include "sensor_msgs/JointState.h"

// Custom : Mesh(for FCL)
#include "sca_pkg/meshes/mesh_utils.hpp"



void rvizCallBack(const sensor_msgs::JointState::ConstPtr& msg);
std::vector<pinocchio::SE3> computeAllPose(const pinocchio::Model model,const pinocchio::Data data, std::vector<std::string> fname_index);

Eigen::VectorXd q_, v_;
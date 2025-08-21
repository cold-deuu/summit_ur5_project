#ifndef __SummitUR5_SCA_Manager__
#define __SummitUR5_SCA_Manager__

//ROS Header
#include "ros/ros.h"
#include <ros/package.h>

//SYSTEM Header
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stdio.h>
#include <tuple>
#include <string>
#include <vector>
#include <ostream>
#include <iostream>
#include <iosfwd>
#include <filesystem>
#include <fstream>

// Linear Algebra Header
#include <Eigen/Dense>
#include <Eigen/QR>



namespace rci_summit_ur5_controller
{
    class SummitUR5_ScaManager
    {
        public:
            SummitUR5_ScaManager();
            ~SummitUR5_ScaManager(){};

            // utils
            Eigen::MatrixXd read_txt_to_matrix(const std::string& filename);
            Eigen::VectorXd tanh(const Eigen::VectorXd& x);
            Eigen::VectorXd softmax(const Eigen::VectorXd& x);
            double forward_pass(const Eigen::VectorXd& q);
            Eigen::MatrixXd numerical_jacobian_gamma(const Eigen::VectorXd& q, double h=1e-6);

        private:
            // Variables
            Eigen::MatrixXd weight1_, weight2_, weight3_, weight4_, weight5_;
            Eigen::VectorXd bias1_, bias2_, bias3_, bias4_, bias5_;
            std::vector<Eigen::MatrixXd> weights_;
            std::vector<Eigen::VectorXd> biases_;



    };
};


#endif
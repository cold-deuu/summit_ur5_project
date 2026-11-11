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
#include <experimental/filesystem>  // gcc 7 (Ubuntu 18.04): no <filesystem>
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
            std::map<std::string, double> forward_pass(const Eigen::VectorXd& q);
            std::map<std::string, Eigen::MatrixXd> numerical_jacobian_gamma(const Eigen::VectorXd& q, double h=1e-6);

        private:
            // Variables
            Eigen::MatrixXd mobile_weight1_, mobile_weight2_, mobile_weight3_, mobile_weight4_, mobile_weight5_;
            Eigen::MatrixXd manip_weight1_, manip_weight2_, manip_weight3_, manip_weight4_, manip_weight5_;
            Eigen::VectorXd mobile_bias1_, mobile_bias2_, mobile_bias3_, mobile_bias4_ ;
            Eigen::VectorXd manip_bias1_, manip_bias2_, manip_bias3_, manip_bias4_;
            std::vector<Eigen::MatrixXd> mobile_weights_, manip_weights_;
            std::vector<Eigen::VectorXd> mobile_biases_, manip_biases_;
            
            // std::map<std::vector<Eigen::MatrixXd>, std::string> J



    };
};


#endif
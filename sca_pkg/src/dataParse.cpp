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

// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include "sca_pkg/meshes/mesh_utils.hpp"
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace Eigen;

void write_txt_data(const vector<VectorXf>& data, const std::string& txt_fname);

int main(int argc, char ** argv)
{
    // ROS
    ros::init(argc, argv, "Data_Processing");
    ros::NodeHandle nh;

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    ros::Rate loop_rate(1); // 

    sensor_msgs::JointState joint_state;
    joint_state.name = {
        "ur5e_shoulder_pan_joint", "ur5e_shoulder_lift_joint", "ur5e_elbow_joint",
        "ur5e_wrist_1_joint", "ur5e_wrist_2_joint","ur5e_wrist_3_joint"
    };


    // Data Save
    std::string mesh_package_path = ros::package::getPath("sca_pkg");
    std::string package_to_data = "/src/data/data_";
    std::string data_path = mesh_package_path + package_to_data;
    std::string bin_fname = data_path+"0"+".bin";
    std::string txt_fname = data_path+"0"+".txt";
    auto binData = read_txt_data(txt_fname);
    int iter = 0;
    while (ros::ok())
    {
        if(iter>binData.size()-1)
        {
            cout<<"Data Processing Finish"<<endl;
            break;
        }
        cout<<"Here"<<endl;
        joint_state.header.stamp = ros::Time::now();
        joint_state.position.resize(6);

        auto pose = binData[binData.size() - 1 - iter];
        if (std::abs(pose[0])<1e-5)
        {
        
            cout<<"Pose : "<<pose.transpose()<<endl;
            for(int i=0; i<6; i++)
            {
                cout<<pose(i+1)<<endl;
                joint_state.position[i] = pose(i+1);
            }
    
            joint_pub.publish(joint_state);    
        }
        iter++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    
    // write_txt_data(binData, txt_fname);
}

void write_txt_data(const vector<VectorXf>& data, const std::string& txt_fname)
{
    ofstream outfile(txt_fname);
    if (!outfile.is_open()) {
        cerr << "Failed to open txt file for writing!" << endl;
        return;
    }

    for (const auto& vec : data) {
        for (int i = 0; i < vec.size(); ++i) {
            outfile << vec[i];
            if (i != vec.size() - 1)
                outfile << " ";
        }
        outfile << "\n";
    }

    outfile.close();
    cout << "Data written to " << txt_fname << endl;
};

#ifndef __Joint_Trajectory_Array_maker__
#define __Joint_Trajectory_Array_maker__

#include <Eigen/QR>    
#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <ros/ros.h>


namespace holistic_controller{
    namespace trajectory{
        class JointCubicTrajectoryArray{
            public:
            JointCubicTrajectoryArray(const std::string & filename, int armjoint_num);
            ros::Time m_stime, m_ctime;
            std::vector<ros::Duration> m_duration_list;
            
            Eigen::VectorXd m_init;
            std::vector<Eigen::VectorXd> m_goal_list;

            //function
            void SetCurrentTime(ros::Time ctime);
            void SetStartTime(ros::Time stime);
            void SetDuration(std::vector<ros::Duration> duration_list);
            void SetInitSample(Eigen::VectorXd init_sample);
            void SetGoalSampleList(std::vector<Eigen::VectorXd> goal_sample_list);
            Eigen::VectorXd computeNext();
            
            int m_iteration;

            protected:
            int m_na;
        };

    }
}

#endif
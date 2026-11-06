#include "rci_holistic_controller/trajectory/joint_cubic_array.hpp"
#include "rci_holistic_controller/math/math.hpp"

using namespace Eigen;
using namespace pinocchio;
using namespace holistic_controller::math;

namespace holistic_controller{
    namespace trajectory{
        JointCubicTrajectoryArray::JointCubicTrajectoryArray(const std::string & filename, int armjoint_num){
            m_na = armjoint_num;
            m_init.setZero(m_na);
            // m_goal.setZero(m_na);
            std::cout<<m_na<<std::endl;

            m_iteration = 0;

        }

        void JointCubicTrajectoryArray::SetCurrentTime(ros::Time ctime){
            m_ctime = ctime;
        }
        
        void JointCubicTrajectoryArray::SetStartTime(ros::Time stime){
            m_stime = stime;
        }

        void JointCubicTrajectoryArray::SetDuration(std::vector<ros::Duration> duration_list){
            m_duration_list = duration_list;
        }
        
        void JointCubicTrajectoryArray::SetInitSample(Eigen::VectorXd init_sample){
            m_init = init_sample;
        }

        void JointCubicTrajectoryArray::SetGoalSampleList(std::vector<Eigen::VectorXd> goal_sample_list){
            m_goal_list = goal_sample_list;
        }

        
        Eigen::VectorXd JointCubicTrajectoryArray::computeNext(){        
            Eigen::VectorXd q_cubic(7);
            
            Eigen::VectorXd init_pose = (m_iteration ==0) ? m_init : m_goal_list[m_iteration];
            q_cubic = init_pose;
            double a0,a1,a2,a3;
            double stime = (m_iteration == 0) ? m_stime.toSec() : m_stime.toSec() + m_duration_list[m_iteration-1].toSec();
            double time = m_ctime.toSec();
            double duration = m_duration_list[m_iteration].toSec();

            if(time<stime){
                return init_pose;
            }
            else if(time> stime + duration){
                for(int i=0;i<q_cubic.size();i++){
                    a0 = init_pose(i);
                    a1 = 0;
                    a2 = 3/pow(duration,2) *(m_goal_list[m_iteration](i)-init_pose(i));
                    a3 = -1.0 *2.0/pow(duration,3) *(m_goal_list[m_iteration](i)-init_pose(i));
                    q_cubic(i) = a0 + a1 * (time - stime) + a2*pow(time-stime,2) + a3 * pow(time-stime,3);
                }
                return q_cubic;
            }
            else {
                m_iteration ++;
                return m_goal_list[m_iteration-1];
            }

        }
    }
}

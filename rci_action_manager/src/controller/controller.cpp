#include "rci_action_manager/controller/controller.hpp"

using namespace Eigen;
using namespace pinocchio;
using namespace std;

using namespace holistic_controller::math;

namespace RobotController
{
    SummitUR5Wrapper::SummitUR5Wrapper(std::shared_ptr<holistic_controller::robot::RobotWrapper> robot, bool is_wholebody)
    : robot_(robot), is_wholebody_(is_wholebody)
    {
        model_ = robot_->model();
        Data data(model_);
        data_ = data;

        Data data_ctrl(model_);
        data_ctrl_ = data_ctrl;


        // na_ = robot_->na();

        // is wholebody 추가
        state_.n_var = model_.nv;
        if (is_wholebody_) state_.n_mobile = 3;
        else state_.n_mobile = 0;
        state_.n_manip = model_.nv -state_.n_mobile;

        ref_.mobile_vel.resize(state_.n_mobile);
        ref_.manip_pos.resize(state_.n_manip);

        ft_sensor_.setZero(6); xdot_admit_.setZero(6);

        // Trajectory
        se3Ref_ = std::make_shared<holistic_controller::trajectory::SE3CubicTrajectory>("SE3_Trajectory");
        jointRef_ = std::make_shared<holistic_controller::trajectory::JointCubicTrajectory>("SE3_Trajectory", state_.n_manip);
        jointArrayRef_ = std::make_shared<holistic_controller::trajectory::JointCubicTrajectoryArray>("Joint_Trajectory_ARRAY", state_.n_manip);

        // Self-Collisoin Library Loading
        summitUr5_scaManager_ = std::make_shared<rci_summit_ur5_controller::SummitUR5_ScaManager>();

        // Task
        se3Task_ = std::make_shared<chanhqp::task::TaskSE3Equality>("se3_task", true);
        jointTask_ = std::make_shared<chanhqp::task::TaskJointPostureEquality>("joint_posture_task");
        scaTask_ = std::make_shared<chanhqp::task::TaskSCAInequality>("sca_task", true);
        baseFixTask_ = std::make_shared<chanhqp::task::TaskBaseFixEquality>("base_fix_task");
        hqp_data_ = std::make_shared<chanhqp::data::HQPData>();
        jointVelLimitTask_ = std::make_shared<chanhqp::task::TaskJointVelocityLimitInequality>("joint_vel_limit_task", false);

        // Joint Velocity Limit of UR5e
        joint_vel_limit_["upper"] = Eigen::VectorXd::Ones(state_.n_var) * 3.0; // URDF : 3.141592
        joint_vel_limit_["lower"] = -Eigen::VectorXd::Ones(state_.n_var) * 3.0; // URDF : 3.141592
        if (state_.n_mobile == 3) {
            joint_vel_limit_["upper"].head(3) << 0.2, 0.2, 0.3;
            joint_vel_limit_["lower"].head(3) << -0.2, -0.2, -0.3;
        }
        ros::NodeHandle limit_params("~");
        double arm_velocity_limit;
        limit_params.param("arm_velocity_limit",arm_velocity_limit,3.0);
        if (!(arm_velocity_limit>0 && arm_velocity_limit<=3.0)) throw std::invalid_argument("Invalid arm_velocity_limit");
        joint_vel_limit_["upper"].tail(state_.n_manip).setConstant(arm_velocity_limit);
        joint_vel_limit_["lower"].tail(state_.n_manip).setConstant(-arm_velocity_limit);
        jointVelLimitTask_ -> setJointVelLimit(joint_vel_limit_);


        
        // 생성자 내부
        // 1. 관성(Mass) 줄이기: 5kg -> 1kg (초기 반응속도가 엄청 빨라집니다)
        M_adm_.setIdentity(); 
        M_adm_.diagonal() << 1.0, 1.0, 1.0, 0.2, 0.2, 0.2; 

        // 2. 댐핑(Damping) 줄이기: 50 -> 10 (꿀단지에서 물속으로 빼냅니다)
        D_adm_.setIdentity();
        D_adm_.diagonal() << 10.0, 10.0, 10.0, 2.0, 2.0, 2.0; 


        K_adm_.setZero(); 
        // K_adm_.setIdentity(); 
        // K_adm_.diagonal() << 25.0, 25.0, 25.0, 5.0, 5.0, 5.0;
        
        x_adm_.setZero();
        xdot_adm_.setZero();
        xddot_adm_.setZero();

        // Input wrench axes match the controlled wrist; shift its sensor origin.
        ros::NodeHandle params("~");
        params.param("wrist_joint",wrist_joint_,std::string("ur5e_wrist_3_joint"));
        if (!model_.existJointName(wrist_joint_)) throw std::invalid_argument("Missing wrist joint");
        params.param("admittance/wrench_sign",wrench_sign_,-1.0);
        if (std::abs(wrench_sign_)!=1.0) throw std::invalid_argument("Invalid wrench_sign");
        sensor_offset_.setZero();
        sensor_offset_(2) = is_wholebody_ ? 0.0 : 0.1;
        params.param("admittance/sensor_offset_x", sensor_offset_(0), 0.0);
        params.param("admittance/sensor_offset_y", sensor_offset_(1), 0.0);
        params.param("admittance/sensor_offset_z", sensor_offset_(2), sensor_offset_(2));
        action_admittance_.parameters.mass = M_adm_.diagonal();
        action_admittance_.parameters.damping = D_adm_.diagonal();
        action_admittance_.parameters.stiffness = K_adm_.diagonal();
        // Optional finite real-robot settings; legacy defaults remain compatible.
        auto read_six=[&](const std::string &name, Eigen::Matrix<double,6,1> &values,bool nonnegative=false) {
            std::vector<double> raw;
            if (!params.getParam("admittance/"+name,raw)) return;
            if(raw.size()!=6) throw std::invalid_argument(name+" must have six entries");
            for(int i=0;i<6;++i) {
                if(!std::isfinite(raw[i]) || (nonnegative?raw[i]<0:raw[i]<=0)) throw std::invalid_argument("Invalid "+name);
                values(i)=raw[i];
            }
        };
        auto &settings=action_admittance_.parameters;
        read_six("mass",settings.mass); read_six("damping",settings.damping);
        read_six("stiffness",settings.stiffness,true); read_six("deadband",settings.deadband,true);
        read_six("velocity_limit",settings.velocity_limit); read_six("acceleration_limit",settings.acceleration_limit);
        read_six("displacement_limit",settings.displacement_limit);
        params.param("admittance/cutoff_hz",settings.cutoff_hz,0.0);
        params.param("admittance/linear_velocity_norm_limit",settings.linear_velocity_norm_limit,settings.linear_velocity_norm_limit);
        params.param("admittance/angular_velocity_norm_limit",settings.angular_velocity_norm_limit,settings.angular_velocity_norm_limit);
        params.param("admittance/smooth_input",smooth_wrench_,false);
        params.param("admittance_enabled",admittance_enabled_,true);
        params.param("admittance/joint_velocity_limit",admittance_joint_velocity_,arm_velocity_limit);
        params.param("admittance/joint_acceleration_limit",admittance_joint_acceleration_,1e9);
        params.param("admittance/idle_reference_lag",idle_reference_lag_,1e9);
        if (!std::isfinite(settings.cutoff_hz) || settings.cutoff_hz<0 ||
            !(settings.linear_velocity_norm_limit>0) || !(settings.angular_velocity_norm_limit>0) ||
            !(admittance_joint_velocity_>0) || !(admittance_joint_acceleration_>0) || !(idle_reference_lag_>0))
            throw std::invalid_argument("Invalid admittance filter/limit parameters");
        wrench_filter_.parameters=settings;
        idle_admittance_.parameters = action_admittance_.parameters;
        std::vector<double> sensor_rpy;
        params.param("admittance/sensor_to_wrist_rpy", sensor_rpy, std::vector<double>{0,0,0});
        if (sensor_rpy.size()!=3) throw std::invalid_argument("sensor_to_wrist_rpy must have 3 entries");
        sensor_rotation_ = (Eigen::AngleAxisd(sensor_rpy[2],Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(sensor_rpy[1],Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(sensor_rpy[0],Eigen::Vector3d::UnitX())).toRotationMatrix();
        reset_action_admittance();


    }

    SE3 SummitUR5Wrapper::oMi() {return state_.oMi;}
    Reference SummitUR5Wrapper::get_result() {return ref_;}
    void SummitUR5Wrapper::initialize()
    {
        // Initialize Global Vector
        state_.q.resize(state_.n_var);
        state_.v.resize(state_.n_var);
        state_.J.resize(6,state_.n_var);

        ref_.mobile_vel = state_.v.head(state_.n_mobile);
        ref_.manip_pos = state_.q.tail(state_.n_manip);


        // Gain Setting
        ros::NodeHandle params("~");
        double kp, kd;
        params.param("se3_kp", kp, 40.0);
        params.param("se3_kd", kd, 2 * sqrt(40.0));
        se3Task_ -> Kp(kp);
        se3Task_ -> Kd(kd);
        double joint_kp; params.param("joint_kp",joint_kp,100.0);
        jointTask_ -> Kp(joint_kp);
    
    }

    void SummitUR5Wrapper::update_joint(Eigen::VectorXd q,  Eigen::VectorXd v)
    {
        // State
        state_.q = q;
        state_.v = v;
        robot_->computeAllTerms(data_, state_.q, state_.v);
        robot_->jacobianLocal(data_, model_.getJointId(wrist_joint_), state_.J);

        
        state_.oMi = robot_->position(data_, model_.getJointId(wrist_joint_));


        state_.eef_vel = state_.J * state_.v;
        if(!telemetry_valid_) { nominal_pose_=reference_pose_=state_.oMi; telemetry_valid_=true; }
        state_.sca_gamma = summitUr5_scaManager_ -> forward_pass(state_.q.tail(state_.n_manip));
        state_.sca_dgamma_dq = summitUr5_scaManager_ -> numerical_jacobian_gamma(state_.q.tail(state_.n_manip));
    }

    void SummitUR5Wrapper::update_ft(Eigen::VectorXd ft)
    {
        ft_sensor_.head(3) = wrench_sign_ * sensor_rotation_ * ft.head(3);
        ft_sensor_.tail(3) = wrench_sign_ * sensor_rotation_ * ft.tail(3);


        wrench_unfiltered_=ft_sensor_;
        if(smooth_wrench_) {
            // Continuous deadband subtraction avoids the old 0 -> 4.5 N jump.
            Eigen::Matrix<double,6,1> filtered;
            wrench_filter_.filter(wrench_unfiltered_,dt_,filtered);
            ft_sensor_=filtered;
            return;
        }
        double force_threshold = 4.5;  
        double torque_threshold = 0.5; 

        if (ft_sensor_.head(3).norm() < force_threshold) {
            ft_sensor_.head(3).setZero();
        }
        if (ft_sensor_.tail(3).norm() < torque_threshold) {
            ft_sensor_.tail(3).setZero();
        }

    }
    void SummitUR5Wrapper::reset_action_admittance()
    {
        idle_active_ = false;
        action_admittance_.reset();
        action_displacement_.setZero();
        action_velocity_.setZero();
        action_rotation_.setIdentity();
        joint_admittance_offset_.setZero(state_.n_manip);
        joint_admittance_qdot_.setZero(state_.n_manip);
        se3Task_->setAdmittance(Eigen::VectorXd::Zero(6));
    }

    Eigen::Matrix<double, 6, 1> SummitUR5Wrapper::update_action_admittance()
    {
        // Integrate in fixed world axes, so wrist rotation does not rotate history.
        const Eigen::Matrix3d &R = state_.oMi.rotation();
        Eigen::Matrix<double, 6, 1> wrench;
        wrench.head<3>() = R * ft_sensor_.head<3>();
        wrench.tail<3>() = R * (ft_sensor_.tail<3>() +
                                  sensor_offset_.cross(ft_sensor_.head<3>()));
        if(!admittance_enabled_)wrench.setZero();
        action_admittance_.step(wrench, dt_);
        action_velocity_ = action_admittance_.velocity;
        action_displacement_ = action_admittance_.displacement;
        action_rotation_ = action_admittance_.rotation;

        // Both tracking tasks use the wrist-local Jacobian.
        Eigen::Matrix<double, 6, 1> local_velocity;
        local_velocity.head<3>() = R.transpose() * action_velocity_.head<3>();
        local_velocity.tail<3>() = R.transpose() * action_velocity_.tail<3>();
        return local_velocity;
    }

    void SummitUR5Wrapper::initJointPosture(VectorXd qTarget, ros::Duration duration)
    {
        ++generation_;
        // Task Remove
        hqp_data_ -> initalize_all_data();

        stime_ = ros::Time::now();
        duration_ =  duration;  

        ref_.manip_pos = state_.q.tail(state_.n_manip);
        ref_.mobile_vel = state_.v.head(state_.n_mobile);
        

        reset_action_admittance();
        jointRef_ -> SetStartTime(ros::Time::now());
        jointRef_ -> SetInitSample(state_.q.tail(state_.n_manip));
        control_mode_="JOINT_POSTURE";
        joint_goal_=qTarget;
        jointRef_ -> SetGoalSample(qTarget);
        jointRef_ -> SetDuration(duration_);

        hqp_data_ -> addTask(jointVelLimitTask_, 0);
        if (state_.n_mobile) hqp_data_->addTask(baseFixTask_, 0);
        hqp_data_ -> addTask(jointTask_, 1);

 


    }
    
    void SummitUR5Wrapper::jointPosture(ros::Time ctime)
    {
        jointRef_->SetCurrentTime(ctime);
        const Eigen::Matrix<double, 6, 1> admittance_velocity = update_action_admittance();
        const Eigen::MatrixXd J = state_.J.rightCols(state_.n_manip);
        // Damped inverse remains bounded near singular joint postures.
        Eigen::VectorXd admittance_qdot = J.transpose() *
            (J * J.transpose() + 0.05 * 0.05 * Eigen::MatrixXd::Identity(6, 6))
                .ldlt().solve(admittance_velocity);
        admittance_qdot=admittance_qdot.cwiseMax(-admittance_joint_velocity_).cwiseMin(admittance_joint_velocity_);
        const double increment=admittance_joint_acceleration_*dt_;
        joint_admittance_qdot_+=(admittance_qdot-joint_admittance_qdot_).cwiseMax(-increment).cwiseMin(increment);
        joint_admittance_offset_ += joint_admittance_qdot_ * dt_;
        const Eigen::VectorXd nominal_q=jointRef_->computeNext();
        nominal_pose_=joint_pose(nominal_q);
        reference_pose_=joint_pose(nominal_q+joint_admittance_offset_);
        jointTask_ -> getVectorTraj(nominal_q + joint_admittance_offset_);
        hqp_data_ -> compute_problem_data(state_);
        Eigen::VectorXd qdot = hqp_data_ -> solve();

        ref_.mobile_vel = qdot.head(state_.n_mobile);
        ref_.manip_pos = state_.q.tail(state_.n_manip) + qdot.tail(state_.n_manip) * dt_; 

        // std::cout<<"[Controller] qdot : \n"<<qdot.tail(state_.n_manip).transpose()<<std::endl;
        // std::cout<<"[Controller] oMi Tra : \n"<<oMi_traj<<std::endl;
        // std::cout<<"[Controller] oMi pos : \n"<<state_.oMi<<std::endl;
        // std::cout<<"[Controller] Joint Tra : \n"<<q_whole.transpose()<<std::endl;
        // std::cout<<"[Controller] manip_pos : \n"<<ref_.manip_pos.transpose()<<std::endl;

    }

    void SummitUR5Wrapper::initMoveitPlanner(trajectory_msgs::JointTrajectory target_traj)
    {
        // saved in points
        // joint_traj
        for(const auto& point : target_traj.points)
        {
            Eigen::Map<const Eigen::VectorXd> traj(point.positions.data(), point.positions.size());
            moveit_traj_.push_back(traj);
        }
        moveit_idx_ = 0;
        moveit_target_idx_ = moveit_traj_.size();
        moveit_succed_ = false;
        moveit_abort_ = false;

        // hqp_data_ -> addTask(scaTask_, 0);
        hqp_data_ -> addTask(jointVelLimitTask_, 0);
        hqp_data_ -> addTask(jointTask_, 1);



    }

    void SummitUR5Wrapper::ControlMoveitPlanner()
    {
        Eigen::VectorXd moveit_target;
        
        if (moveit_idx_ <moveit_target_idx_)
        {
            moveit_target = moveit_traj_[moveit_idx_];
        }
        else{
            moveit_target = moveit_traj_.back();
        }
        
        moveit_idx_ ++;

        // Test 1
        ref_.manip_pos = moveit_target;

        if (((state_.q.tail(state_.n_manip) - moveit_target).norm() < 1e-3) && (moveit_idx_ >= moveit_target_idx_))
        {
            moveit_succed_ = true;
        }

        else if (moveit_idx_ > moveit_target_idx_ + 300)
        {
            moveit_abort_ = true;
        }

        std::cout<<"[Controller] Manipulator Desired : "<<ref_.manip_pos.transpose()<<std::endl;

        // Test 2
        // jointTask_->getVectorTraj(moveit_target);
        // hqp_data_ -> compute_problem_data();
        // Eigen::VectorXd qdot = hqp_data_->solve();
        
        // ref_.manip_pos = state_.q.tail(state_.n_manip) + qdot.tail(state_.n_manip) * dt_;
    }


    void SummitUR5Wrapper::initControl(SE3 target, ros::Duration duration, bool rel)
    {
        ++generation_;
        // Task Remove
        hqp_data_ -> initalize_all_data();

        stime_ = ros::Time::now();
        duration_ = duration;
        
        SE3 oMg;
        if(rel)
        {
            oMg.translation() = state_.oMi.translation() + target.translation();
            oMg.rotation() = state_.oMi.rotation() * target.rotation();
        }
        else oMg = target;

        ref_.manip_pos = state_.q.tail(state_.n_manip);
        ref_.mobile_vel = state_.v.head(state_.n_mobile);

        reset_action_admittance();
        se3Ref_ -> SetInitSample(state_.oMi);
        control_mode_="SE3";
        se3_goal_=oMg;
        se3Ref_ -> SetGoalSample(oMg);
        se3Ref_ -> SetStartTime(stime_);
        se3Ref_ -> SetDuration(duration);

        // Task Insert
        // hqp_data_ -> addTask(scaTask_, 0);
        hqp_data_ -> addTask(jointVelLimitTask_, 0);
        hqp_data_ -> addTask(se3Task_, 1);
        // hqp_data_ -> addTask(baseFixTask_, 2);        
        std::cout<<"[Controller] Goal SE3 : \n "<<oMg<<std::endl;


    }

    // void SummitUR5Wrapper::Control(ros::Time ctime)
    // {
    //     se3Ref_ -> SetCurrentTime(ctime);    
    //     se3Task_ -> getSE3Traj(se3Ref_->computeNext());

    //     hqp_data_ -> compute_problem_data(state_);
    //     Eigen::VectorXd qdot = hqp_data_ -> solve();

    //     ref_.mobile_vel = qdot.head(state_.n_mobile);
    //     ref_.manip_pos = state_.q.tail(state_.n_manip) + qdot.tail(state_.n_manip) * dt_; 
    //     std::cout<<"[Controller] Current SE3 : \n "<<state_.oMi<<std::endl;

    //     // std::cout<<"[Controller] manip_pos : \n"<<ref_.manip_pos.transpose()<<std::endl;

    // }

    void SummitUR5Wrapper::Control(ros::Time ctime)
    {
        // 1. 기존의 원래 가려던 목표 궤적(Reference) 가져오기
        se3Ref_->SetCurrentTime(ctime);    
        const Eigen::Matrix<double, 6, 1> admittance_velocity = update_action_admittance();
        nominal_pose_=se3Ref_->computeNext();
        SE3 compliant_reference = nominal_pose_;
        compliant_reference.translation() += action_displacement_.head<3>();
        compliant_reference.rotation() = action_rotation_ * compliant_reference.rotation();
        reference_pose_=compliant_reference;
        se3Task_->getSE3Traj(compliant_reference);
        se3Task_->setAdmittance(admittance_velocity);
        hqp_data_->compute_problem_data(state_);
        Eigen::VectorXd qdot = hqp_data_->solve();

        ref_.mobile_vel = qdot.head(state_.n_mobile);
        
        Eigen::VectorXd v_step = Eigen::VectorXd::Zero(model_.nv);
        v_step.tail(state_.n_manip) = qdot.tail(state_.n_manip) * dt_; 
        Eigen::VectorXd q_next = Eigen::VectorXd::Zero(model_.nq);
        // Resolved-rate tracking is one HQP velocity step from measured joints.
        // Integrating the previous command instead accumulates servo lag and
        // can violate the measured-to-command step bound despite bounded qdot.
        pinocchio::integrate(model_, state_.q, v_step, q_next);
        
        ref_.manip_pos = q_next.tail(state_.n_manip); 

        // std::cout << "[Controller] Current SE3 : \n " << state_.oMi << std::endl;
        // std::cout << "[Controller] Admittance Offset : " << x_adm_.head(3).transpose() << std::endl;
    }

    SE3 SummitUR5Wrapper::joint_pose(const Eigen::VectorXd &arm_q)
    {
        Eigen::VectorXd q=state_.q;q.tail(state_.n_manip)=arm_q;
        pinocchio::forwardKinematics(model_,data_ctrl_,q);
        return data_ctrl_.oMi[model_.getJointId(wrist_joint_)];
    }

    bool SummitUR5Wrapper::telemetry(rci_action_manager::ControlTelemetry &out) const
    {
        if(!telemetry_valid_) return false;
        auto pose=[](const SE3 &value,geometry_msgs::Pose &p) {
            p.position.x=value.translation().x();p.position.y=value.translation().y();p.position.z=value.translation().z();
            const Eigen::Quaterniond q(value.rotation());
            p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();
        };
        out.mode=control_mode_;out.action_id=generation_;
        pose(state_.oMi,out.actual_pose);pose(nominal_pose_,out.nominal_pose);pose(reference_pose_,out.reference_pose);
        const Eigen::Vector3d translation=reference_pose_.translation()-state_.oMi.translation();
        const Eigen::Vector3d rotation=pinocchio::log3(Eigen::Matrix3d(reference_pose_.rotation()*state_.oMi.rotation().transpose()));
        const auto &admittance=idle_active_?idle_admittance_:action_admittance_;
        for(int i=0;i<3;++i) { out.translation_error[i]=translation(i);out.rotation_error[i]=rotation(i); }
        for(int i=0;i<6;++i) {
            out.joint_actual[i]=state_.q(state_.n_mobile+i);out.joint_command[i]=ref_.manip_pos(i);
            out.joint_error[i]=out.joint_command[i]-out.joint_actual[i];
            out.wrench_bias_corrected[i]=wrench_unfiltered_(i);out.wrench_filtered[i]=ft_sensor_(i);
            out.admittance_displacement[i]=admittance.displacement(i);out.admittance_velocity[i]=admittance.velocity(i);
        }
        return true;
    }

    bool SummitUR5Wrapper::se3_settled(double position_tolerance,double rotation_tolerance) const
    {
        const Eigen::Vector3d position=se3_goal_.translation()+action_displacement_.head<3>();
        const Eigen::Matrix3d rotation=action_rotation_*se3_goal_.rotation();
        return (state_.oMi.translation()-position).norm()<position_tolerance &&
            pinocchio::log3(state_.oMi.rotation().transpose()*rotation).norm()<rotation_tolerance &&
            state_.eef_vel.norm()<.03;
    }

    void SummitUR5Wrapper::update_admittance_control()
    {
        if (!idle_active_) {
            ref_.manip_pos = state_.q.tail(state_.n_manip);
            nominal_pose_=state_.oMi;
            idle_qdot_=Eigen::VectorXd::Zero(state_.n_manip);
            idle_admittance_.reset();
            idle_active_ = true;
        }
        control_mode_="HOLDING";
        const Eigen::Matrix3d &R=state_.oMi.rotation();
        Eigen::Matrix<double,6,1> f_ext;
        f_ext.head<3>()=R*ft_sensor_.head<3>();
        f_ext.tail<3>()=R*(ft_sensor_.tail<3>()+sensor_offset_.cross(ft_sensor_.head<3>()));
        if(!admittance_enabled_)f_ext.setZero();
        idle_admittance_.step(f_ext, dt_);
        xdot_adm_ = idle_admittance_.velocity;
        x_adm_ = idle_admittance_.displacement;

        // Idle admittance keeps the mobile base stationary and controls the arm.
        Eigen::MatrixXd J = state_.J.rightCols(state_.n_manip);
        
        // Moore-Penrose
        // Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();

        double lambda = 0.05; 
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd J_pinv = J.transpose() * (J * J.transpose() + lambda * lambda * I).inverse();

        Eigen::Matrix<double,6,1> local_velocity;
        local_velocity.head<3>()=R.transpose()*xdot_adm_.head<3>();
        local_velocity.tail<3>()=R.transpose()*xdot_adm_.tail<3>();
        Eigen::VectorXd qdot_cmd = J_pinv * local_velocity;
        qdot_cmd=qdot_cmd.cwiseMax(-admittance_joint_velocity_).cwiseMin(admittance_joint_velocity_);
        qdot_cmd=qdot_cmd.cwiseMax(joint_vel_limit_.at("lower").tail(state_.n_manip))
                           .cwiseMin(joint_vel_limit_.at("upper").tail(state_.n_manip));

        const double increment=admittance_joint_acceleration_*dt_;
        idle_qdot_+=(qdot_cmd-idle_qdot_).cwiseMax(-increment).cwiseMin(increment);
        Eigen::VectorXd v_step = Eigen::VectorXd::Zero(model_.nv);
        v_step.tail(state_.n_manip) = idle_qdot_ * dt_; 
        
        Eigen::VectorXd q_next = Eigen::VectorXd::Zero(model_.nq);
        // Integrate the held reference, not measured contact/servo drift.
        Eigen::VectorXd held_q = state_.q;
        // Anti-windup: never store arbitrarily large held-reference lag while
        // hardware is delayed or physically constrained by the operator.
        held_q.tail(state_.n_manip) = state_.q.tail(state_.n_manip)+
            (ref_.manip_pos-state_.q.tail(state_.n_manip)).cwiseMax(-idle_reference_lag_).cwiseMin(idle_reference_lag_);
        pinocchio::integrate(model_, held_q, v_step, q_next);

        ref_.manip_pos = q_next.tail(state_.n_manip);
        reference_pose_=joint_pose(ref_.manip_pos);
        ref_.mobile_vel.setZero(state_.n_mobile); 
    }


}



#include "rci_action_manager/controller/controller.hpp"
#include <ros/package.h>
#include <stdexcept>

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

void check_mode(bool whole_body, bool joint_mode)
{
    const std::string package = ros::package::getPath("summit_xl_description");
    auto robot = std::make_shared<holistic_controller::robot::RobotWrapper>(
        package + (whole_body ? "/summit_ur_wholebody_control.urdf" :
                                "/summit_ur_manip_control.urdf"),
        std::vector<std::string>{package});
    RobotController::SummitUR5Wrapper controller(robot, whole_body);
    controller.initialize();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(robot->nq());
    q.tail(6) << M_PI, -1.57, 1.57, -1.57, -1.57, 0.0;
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(robot->nv());
    const Eigen::VectorXd no_force = Eigen::VectorXd::Zero(6);
    controller.update_joint(q, zero);
    const auto start = controller.oMi();
    const auto begin = [&]() {
        if (joint_mode) controller.initJointPosture(q.tail(6), ros::Duration(2));
        else controller.initControl(pinocchio::SE3::Identity(), ros::Duration(2), true);
    };
    const auto compute = [&]() {
        if (joint_mode) controller.jointPosture(ros::Time::now());
        else controller.Control(ros::Time::now());
        const auto command = controller.get_result();
        require(command.manip_pos.size() == 6, "Wrong manipulator command dimension");
        require(command.mobile_vel.size() == (whole_body ? 3 : 0), "Wrong mobile command dimension");
        require(command.manip_pos.allFinite() && command.mobile_vel.allFinite(), "Nonfinite command");
        require(((command.manip_pos - q.tail(6)).array().abs() <= 0.030001).all(),
                "Admittance bypassed HQP joint velocity limits");
        return command;
    };

    controller.update_ft(no_force);
    begin();
    auto reference = compute();
    require((reference.manip_pos - q.tail(6)).norm() < 1e-9, "Zero-force tracking changed");

    // Sensor sign convention: negative sensor Fz means positive external wrist Fz.
    Eigen::VectorXd wrench = no_force;
    wrench(2) = -6;
    controller.update_ft(wrench);
    reference = compute();
    require((reference.manip_pos - q.tail(6)).norm() > 1e-5, "Force caused no compliance");
    q.tail(6) = reference.manip_pos;
    if (whole_body) q.head(3) += reference.mobile_vel * 0.01;
    controller.update_joint(q, zero);
    require((controller.oMi().translation() - start.translation()).dot(start.rotation().col(2)) > 0,
            "Compliance moved against the applied force");

    // A new action must not inherit the previous action's offset or velocity.
    controller.update_ft(no_force);
    begin();
    reference = compute();
    require((reference.manip_pos - q.tail(6)).norm() < 1e-9, "Action retained stale compliance");

    const Eigen::Matrix3d before_torque = controller.oMi().rotation();
    wrench.setZero();
    wrench(5) = -1;
    controller.update_ft(wrench);
    reference = compute();
    require((reference.manip_pos - q.tail(6)).norm() > 1e-5, "Torque caused no compliance");
    q.tail(6) = reference.manip_pos;
    if (whole_body) q.head(3) += reference.mobile_vel * 0.01;
    controller.update_joint(q, zero);
    require(pinocchio::log3(Eigen::Matrix3d(
                before_torque.transpose() * controller.oMi().rotation()))(2) > 0,
            "Compliance rotated against the applied torque");

    // Even a very large requested compliance velocity must pass through HQP.
    wrench.setZero();
    wrench(2) = -6000;
    controller.update_ft(wrench);
    compute();
}

void check_se3_lag_bound(bool whole_body)
{
    ros::NodeHandle params("~"); params.setParam("arm_velocity_limit",1.57);
    const std::string package=ros::package::getPath("summit_xl_description");
    auto robot=std::make_shared<holistic_controller::robot::RobotWrapper>(
        package+(whole_body?"/summit_ur_wholebody_control.urdf":"/summit_ur_manip_control.urdf"),
        std::vector<std::string>{package});
    RobotController::SummitUR5Wrapper controller(robot,whole_body);controller.initialize();
    Eigen::VectorXd q=Eigen::VectorXd::Zero(robot->nq());
    q.tail(6)<<.01,-1.47,1.0,-1.57,-.7,.002;
    const Eigen::VectorXd zero=Eigen::VectorXd::Zero(robot->nv());
    controller.update_joint(q,zero);controller.update_ft(Eigen::VectorXd::Zero(6));
    pinocchio::SE3 target=pinocchio::SE3::Identity();target.translation()<<.2,0,-.2;
    controller.initControl(target,ros::Duration(3),true);
    const ros::Time start=ros::Time::now();
    // Hold measured joints fixed: an unresponsive/lagging plant must never make
    // SE3 position commands accumulate beyond one bounded HQP velocity step.
    for(int i=0;i<300;++i) {
        controller.update_joint(q,zero);
        controller.Control(start+ros::Duration(.01*i));
        const auto result=controller.get_result();
        require(result.manip_pos.allFinite(),"SE3 lag produced nonfinite command");
        require((result.manip_pos-q.tail(6)).cwiseAbs().maxCoeff()<=1.570001*.01,
                "SE3 command accumulated over measured joint state during lag");
    }
    params.deleteParam("arm_velocity_limit");
}

void check_smooth_admittance(bool whole_body)
{
    ros::NodeHandle params("~");
    params.setParam("admittance/smooth_input",true);
    params.setParam("admittance/mass",std::vector<double>{5,5,5,.5,.5,.5});
    params.setParam("admittance/damping",std::vector<double>{60,60,60,6,6,6});
    params.setParam("admittance/deadband",std::vector<double>{2,2,2,.15,.15,.15});
    params.setParam("admittance/cutoff_hz",4.0);
    params.setParam("admittance/velocity_limit",std::vector<double>{.02,.02,.02,.08,.08,.08});
    params.setParam("admittance/acceleration_limit",std::vector<double>{.15,.15,.15,.5,.5,.5});
    params.setParam("admittance/displacement_limit",std::vector<double>{.1,.1,.1,.3,.3,.3});
    params.setParam("admittance/linear_velocity_norm_limit",.03);
    params.setParam("admittance/angular_velocity_norm_limit",.1);
    params.setParam("admittance/joint_velocity_limit",.15);
    params.setParam("admittance/joint_acceleration_limit",.4);
    params.setParam("admittance/idle_reference_lag",.005);
    const std::string package=ros::package::getPath("summit_xl_description");
    auto robot=std::make_shared<holistic_controller::robot::RobotWrapper>(
        package+(whole_body?"/summit_ur_wholebody_control.urdf":"/summit_ur_manip_control.urdf"),
        std::vector<std::string>{package});
    RobotController::SummitUR5Wrapper controller(robot,whole_body);controller.initialize();
    Eigen::VectorXd q=Eigen::VectorXd::Zero(robot->nq());q.tail(6)<<.01,-1.47,1.0,-1.57,-.7,.002;
    const Eigen::VectorXd zero=Eigen::VectorXd::Zero(robot->nv());
    Eigen::Vector3d previous=Eigen::Vector3d::Zero();
    double maximum_gap=0,maximum_velocity=0;
    for(int i=0;i<500;++i) {
        controller.update_joint(q,zero);
        Eigen::VectorXd force=Eigen::VectorXd::Zero(6);
        if(i>10 && i<110) force(2)=-200; // abrupt strong push; measured robot frozen
        controller.update_ft(force);controller.update_admittance_control();
        const auto command=controller.get_result();
        const double gap=(command.manip_pos-q.tail(6)).cwiseAbs().maxCoeff();
        require(gap<=.006501,"Idle reference accumulated while hardware frozen");
        maximum_gap=std::max(maximum_gap,gap);
        rci_action_manager::ControlTelemetry t;require(controller.telemetry(t),"Missing telemetry");
        Eigen::Vector3d velocity(t.admittance_velocity[0],t.admittance_velocity[1],t.admittance_velocity[2]);
        require(velocity.norm()<=.030001,"Admittance speed norm limit exceeded");
        require((velocity-previous).norm()<=std::sqrt(3.)*.15*.01+1e-7,"Admittance impulse bypassed acceleration bound");
        require(std::isfinite(t.rotation_error[0]) && std::isfinite(t.translation_error[0]),"Invalid pose/error telemetry");
        previous=velocity;maximum_velocity=std::max(maximum_velocity,velocity.norm());
    }
    require(previous.norm()<1e-6,"Admittance velocity did not decay after release");
    std::cout<<"smooth_admittance whole_body="<<whole_body<<" max_joint_gap="<<maximum_gap
             <<" max_linear_velocity="<<maximum_velocity<<std::endl;
    for(const std::string &key:{"smooth_input","mass","damping","deadband","cutoff_hz","velocity_limit", "acceleration_limit",
        "displacement_limit","linear_velocity_norm_limit","angular_velocity_norm_limit","joint_velocity_limit",
        "joint_acceleration_limit","idle_reference_lag"}) params.deleteParam("admittance/"+key);
}

void check_moveit_admittance_limits(bool whole_body, const std::string &mode)
{
    ros::NodeHandle params("~");
    params.setParam("admittance_enabled",true);
    params.setParam("admittance/displacement_limits_enabled",false);
    params.setParam("admittance/acceleration_limits_enabled",false);
    // Tiny finite bounds expose any Cartesian or joint slew limit left active.
    params.setParam("admittance/displacement_limit",std::vector<double>(6,1e-6));
    params.setParam("admittance/acceleration_limit",std::vector<double>(6,1e-6));
    params.setParam("admittance/joint_acceleration_limit",1e-6);
    params.setParam("admittance/velocity_limit",std::vector<double>{.06,.06,.06,.08,.08,.08});
    params.setParam("admittance/linear_velocity_norm_limit",.08);
    params.setParam("admittance/angular_velocity_norm_limit",.1);
    params.setParam("admittance/joint_velocity_limit",.15);
    params.setParam("admittance/idle_reference_lag",.005);
    const std::string package=ros::package::getPath("summit_xl_description");
    auto robot=std::make_shared<holistic_controller::robot::RobotWrapper>(
        package+(whole_body?"/summit_ur_wholebody_control.urdf":"/summit_ur_manip_control.urdf"),
        std::vector<std::string>{package});
    RobotController::SummitUR5Wrapper controller(robot,whole_body);controller.initialize();
    Eigen::VectorXd q=Eigen::VectorXd::Zero(robot->nq());
    q.tail(6)<<.01,-1.47,1.0,-1.57,-.7,.002;
    const Eigen::VectorXd zero=Eigen::VectorXd::Zero(robot->nv());
    controller.update_joint(q,zero);
    if (mode=="JOINT_POSTURE") controller.initJointPosture(q.tail(6),ros::Duration(2));
    else if (mode=="SE3") controller.initControl(pinocchio::SE3::Identity(),ros::Duration(2),true);
    Eigen::VectorXd wrench=Eigen::VectorXd::Zero(6);
    wrench(2)=-200; wrench(5)=-10;
    const ros::Time start=ros::Time::now();
    for(int i=0;i<40;++i) {
        controller.update_joint(q,zero);controller.update_ft(wrench);
        if (mode=="HOLDING") controller.update_admittance_control();
        else if (mode=="JOINT_POSTURE") controller.jointPosture(start+ros::Duration(.01*i));
        else controller.Control(start+ros::Duration(.01*i));
        rci_action_manager::ControlTelemetry t;
        require(controller.telemetry(t),"Missing unlimited admittance telemetry");
        Eigen::Vector3d linear,angular,displacement;
        for(int axis=0;axis<3;++axis) {
            linear(axis)=t.admittance_velocity[axis];
            angular(axis)=t.admittance_velocity[axis+3];
            displacement(axis)=t.admittance_displacement[axis];
        }
        require(linear.norm()>.03,"Admittance acceleration clipping was not disabled");
        require(displacement.norm()>1e-4,"Admittance displacement clipping was not disabled");
        require(linear.norm()<=.080001 && linear.cwiseAbs().maxCoeff()<=.060001,
                "Disabling acceleration/displacement limits removed linear speed limits");
        require(angular.norm()<=.100001 && angular.cwiseAbs().maxCoeff()<=.080001,
                "Disabling acceleration/displacement limits removed angular speed limits");
        const auto command=controller.get_result();
        require(command.manip_pos.allFinite(),"Unlimited admittance produced nonfinite command");
        const double step=(command.manip_pos-q.tail(6)).cwiseAbs().maxCoeff();
        require(step>1e-5,"Admittance joint acceleration clipping was not disabled");
        require(step<=.030001,"Unlimited admittance bypassed arm velocity limits");
    }
    for(const std::string &key:{"displacement_limits_enabled","acceleration_limits_enabled",
        "displacement_limit","acceleration_limit","joint_acceleration_limit","velocity_limit",
        "linear_velocity_norm_limit","angular_velocity_norm_limit","joint_velocity_limit",
        "idle_reference_lag"}) params.deleteParam("admittance/"+key);
    params.deleteParam("admittance_enabled");
    std::cout<<"moveit_admittance_limits whole_body="<<whole_body<<" mode="<<mode<<" passed"<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "admittance_controller_test", ros::init_options::AnonymousName);
    check_mode(false, true);
    check_mode(false, false);
    check_mode(true, true);
    check_mode(true, false);
    check_se3_lag_bound(false);
    check_se3_lag_bound(true);
    check_smooth_admittance(false);
    check_smooth_admittance(true);
    for(bool whole_body:{false,true})
        for(const std::string &mode:{"HOLDING","JOINT_POSTURE","SE3"})
            check_moveit_admittance_limits(whole_body,mode);
    return 0;
}

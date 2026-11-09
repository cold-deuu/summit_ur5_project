#pragma once
#include <rci_holistic_controller/data/hqp_data.hpp>
#include <rci_holistic_controller/task/task_joint_posture_equality.hpp>
#include <rci_holistic_controller/admittance/cartesian_admittance.hpp>
#include <atomic>
#include <array>
#include <cstring>
#include <vector>

namespace moveit_optimal {
using Vec = Eigen::Matrix<double,6,1>;
using Mat = Eigen::Matrix<double,6,6>;

// Single writer, bounded nonblocking reads. Atomics avoid the data race of an
// ordinary seqlock on a non-atomic struct. No shared_ptr deletion on the RT thread.
template<class T> class AtomicValue {
    static constexpr size_t N = (sizeof(T)+7)/8;
    std::array<std::atomic<uint64_t>,N> words_;
    std::atomic<uint64_t> sequence_{0};
public:
    AtomicValue() { for (auto &w: words_) w.store(0); }
    void store(const T &value) {
        std::array<uint64_t,N> copy{};
        std::memcpy(copy.data(), &value, sizeof(T));
        sequence_.fetch_add(1, std::memory_order_seq_cst);
        for(size_t i=0;i<N;++i) words_[i].store(copy[i],std::memory_order_seq_cst);
        sequence_.fetch_add(1, std::memory_order_seq_cst);
    }
    bool load(T &value) const {
        for(int attempt=0;attempt<3;++attempt) {
            const auto seq=sequence_.load(std::memory_order_seq_cst);
            if(seq&1) continue;
            std::array<uint64_t,N> copy;
            for(size_t i=0;i<N;++i) copy[i]=words_[i].load(std::memory_order_seq_cst);
            if(seq==sequence_.load(std::memory_order_seq_cst)) {
                std::memcpy(&value,copy.data(),sizeof(T)); return true;
            }
        }
        return false;
    }
};

// SERVOING: [BRL_lab] streamed Cartesian target (servo_enabled), see Controller::servoReference.
enum class State { HOLDING, TRACKING, SERVOING };
enum class Result { NONE, SUCCEEDED, REJECTED, ABORTED };
enum class Reason { NONE, GOAL_REACHED, INVALID_NAMES, INVALID_POINTS, INVALID_TIME,
    START_MISMATCH, STALE_STAMP, BUSY, REPLACED, PATH_TOLERANCE, GOAL_TOLERANCE,
    STATE_TIMEOUT, FT_TIMEOUT, QP_FAILURE, JOINT_LIMIT, NOT_READY, INVALID_LIMITS, DRIVER_NOT_READY, COMMAND_GAP,
    SERVO_ENDED, SERVO_WORKSPACE };
const char *reasonText(Reason);
struct JointState { std::array<double,6> q{}, v{}; double stamp=0; bool valid=false; };
// [BRL_lab] Servo target for servo_frame in the model root frame: x y z qx qy qz qw.
// One id per uninterrupted stream; stamp = receive time (ros::Time, same clock as step()).
struct ServoTarget { std::array<double,7> pose{}; double stamp=0; uint64_t id=0; bool valid=false; };
struct Wrench { std::array<double,6> value{}; double stamp=0; bool ready=false; };
struct Snapshot {
    std::array<double,6> actual{},desired{},command{},velocity{},error{},displacement{},admittance_velocity{};
    std::array<double,7> actual_pose{},nominal_pose{},reference_pose{};
    std::array<double,3> translation_error{},rotation_error{};
    std::array<double,6> wrench_bias_corrected{},wrench_filtered{};
    State state=State::HOLDING; Result result=Result::NONE; Reason reason=Reason::NONE;
    uint64_t trajectory_id=0, result_id=0, sequence=0, missed_deadlines=0;
    double stamp=0,elapsed=0,rms=0,max_error=0,nominal_error=0,loop_max_ms=0;
    bool initialized=false, command_valid=false, bias_ready=false,telemetry_valid=false;
};
struct Sample { Vec q=Vec::Zero(), v=Vec::Zero(), a=Vec::Zero(); };
struct Segment {
    double start=0,duration=0;
    Eigen::Matrix<double,6,6> coefficients=Eigen::Matrix<double,6,6>::Zero();
    static Segment make(double start,double duration,const Sample&,const Sample&);
    Sample sample(double t) const;
};
struct Plan {
    struct PathPoint { Vec q; double distance,parameter; };
    std::vector<Segment> segments; // capacity allocated at startup, never resized in RT
    std::vector<PathPoint> path; // Prepared by the non-RT trajectory callback.
    Sample first,last;
    double start_time=0,duration=0;
    double parameter_scale=1;
    bool timed=false; // TOPP-RA retimed: elapsed time is the path parameter.
    uint64_t id=0;
    Sample sample(double elapsed) const;
    bool buildPath();
    Sample follow(const Vec &actual,double &progress,double lookahead,double window) const;
    double parameterAtDistance(double distance) const;
    Sample sampleProgress(double progress) const;
};
// Seven-phase S-curve: accelerate, cruise, brake only at the real endpoint.
class PathProgressGovernor {
public:
    double position=0,velocity=0,acceleration=0;
    void reset();
    void startCruise(double goal,double max_velocity,double max_acceleration,double max_jerk);
    void advance(double dt);
private:
    std::array<double,7> durations_{},jerks_{};
    double elapsed_=0,goal_=0;
    size_t phase_=7;
};
// Smoothly slow the path clock when measured tracking lag grows. Exact
// critically damped update preserves rate and its derivative across updates.
class PathClock {
public:
    double rate=1,derivative=0;
    void reset() {rate=1;derivative=0;}
    double advance(double target,double dt,double bandwidth);
};
pinocchio::Model armModel(const pinocchio::Model&,const std::array<std::string,6>&);
struct Parameters;
// Non-RT: replace plan segments with a C2 TOPP-RA time parameterization.
bool retimePlan(Plan &plan,const Parameters &p,std::string &error);
struct Parameters {
    std::string urdf;
    std::string wrist_joint="ur5e_wrist_3_joint";
    std::array<std::string,6> joint_names;
    double rate=100,kp=12,regularization=1e-5;
    double start_tolerance=.05,path_tolerance=.25,goal_tolerance=.015;
    double goal_velocity_tolerance=.025,goal_timeout=3,transition_duration=.4;
    double state_timeout=.25,ft_timeout=.25,position_margin=.005;
    Vec lower,upper,velocity,acceleration,effort;
    bool torque_limits=false,tracking_admittance=true,hold_mode=false;
    bool admittance_enabled=true;
    bool geometric_tracking=false;
    // [BRL_lab] Judge path progress/goal on q minus the admittance joint offset, so an
    // intentional push during tracking does not stall the path clock (default off = field).
    bool admittance_aware_progress=false;
    bool acceleration_limits=true;
    double path_lookahead=.03,path_search_window=.12,path_stall_timeout=15;
    double path_speed=.5,path_acceleration=2,path_jerk=10;
    double path_clock_bandwidth=10;
    // Non-RT TOPP-RA retiming of geometric paths; falls back to the governor on failure.
    bool path_retiming=false;
    double retiming_velocity_scale=.9,retiming_acceleration_scale=.9,retiming_jerk_scale=.8,retiming_sample_period=.02;
    // Joint jerk bound for the retimed reference (scaled by retiming_jerk_scale).
    Vec jerk=Vec::Constant(10);
    size_t max_points=5000;
    bool require_driver_ready=false,limit_joint_admittance=false,latch_faults=false;
    double admittance_joint_velocity=.15,admittance_joint_acceleration=.4,holding_reference_lag=.005;
    double max_command_gap=1e9;
    holistic_controller::admittance::CartesianAdmittance::Parameters admittance;
    Vec tracking_stiffness=Vec::Constant(100),holding_stiffness=Vec::Constant(100);
    Eigen::Vector3d sensor_offset=Eigen::Vector3d::Zero();
    Eigen::Matrix3d sensor_rotation=Eigen::Matrix3d::Identity();
    double wrench_sign=-1;
    // [BRL_lab] Cartesian servo (used only when targets arrive; the topic node subscribes
    // only with ~servo_enabled). The reference posture moves by damped differential IK of
    // servo_frame toward the latest target: twist = gain * pose error, norm-clamped to the
    // speeds, rate-limited by the accelerations, then the usual HQP joint tracking follows it.
    std::string servo_frame="graps_link";
    double servo_gain=1.5,servo_linear_speed=.05,servo_angular_speed=.2;
    double servo_linear_acceleration=.2,servo_angular_acceleration=.8;
    double servo_timeout=.3,servo_damping=.01,servo_max_travel=.25;
};
class Controller {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit Controller(const Parameters&);
    AtomicValue<JointState> joint_input;
    AtomicValue<Wrench> wrench_input;
    AtomicValue<Snapshot> output;
    AtomicValue<ServoTarget> servo_input; // [BRL_lab] single writer: topic node servo callback
    bool servoAvailable() const { return servo_available_; }
    std::atomic<bool> hold_mode,tracking_admittance;
    std::atomic<bool> driver_running{true};
    std::atomic<double> driver_scale{1.0},driver_scaling_stamp{0.0};
    // One serialized non-RT callback writes plans; step() is the sole RT consumer.
    int acquirePlan();
    Plan &plan(int slot) { return plans_[slot]; }
    void submitPlan(int slot);
    void releasePlan(int slot);
    void reject(Reason reason,uint64_t id);
    void step(double now);
    void timing(double duration_ms,bool missed);
private:
    Parameters p_;
    std::array<Plan,3> plans_;
    std::array<std::atomic<int>,3> slots_; // FREE=0, WRITING=1, PENDING=2, ACTIVE=3
    std::atomic<int> pending_{-1};
    std::atomic<int> rejection_{0};
    std::atomic<uint64_t> rejected_id_{0};
    int active_=-1;
    bool started_=false;
    double start_=0,dt_,error_sum_=0,plan_elapsed_=0,last_step_wall_=0;
    double path_progress_=0,path_last_progress_=0,path_progress_stamp_=0;
    double path_handover_elapsed_=0,path_time_=0;
    PathProgressGovernor path_reference_;
    PathClock path_clock_;
    bool path_cruise_started_=false;
    Reason fault_=Reason::NONE;
    size_t error_count_=0;
    std::shared_ptr<holistic_controller::robot::RobotWrapper> robot_;
    pinocchio::Model model_;
    pinocchio::Data actual_data_,nominal_data_,reference_data_,servo_data_;
    pinocchio::JointIndex wrist_;
    // [BRL_lab] servo session state (RT thread only)
    bool servo_available_=false,servo_active_=false;
    pinocchio::FrameIndex servo_frame_=0;
    uint64_t servo_id_=0;
    ServoTarget servo_;
    Eigen::Vector3d servo_origin_=Eigen::Vector3d::Zero();
    Vec servo_twist_=Vec::Zero();
    Mat servo_jacobian_=Mat::Zero();
    void startServo(const ServoTarget&,double now);
    void servoReference(double now,double scale);
    void endServo(Result,Reason);
    void abortMotion(Reason);
    chanhqp::data::HQPData hqp_;
    chanhqp::task::TaskJointPostureEquality task_;
    holistic_controller::admittance::CartesianAdmittance admittance_;
    JointState joints_;
    Wrench wrench_;
    Snapshot status_;
    Sample nominal_;
    Segment blend_;
    Vec command_=Vec::Zero(),previous_velocity_=Vec::Zero(),hold_=Vec::Zero();
    Vec joint_offset_=Vec::Zero(),offset_velocity_=Vec::Zero();
    Mat jacobian_=Mat::Zero(),mass_=Mat::Identity();
    void finish(Result,Reason);
};
}

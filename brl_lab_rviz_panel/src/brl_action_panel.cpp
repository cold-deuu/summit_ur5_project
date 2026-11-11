#include "brl_lab_rviz_panel/brl_action_panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QTimer>
#include <QApplication>
#include <QGridLayout>
#include <std_srvs/Trigger.h>
#include <QVBoxLayout>
#include <cmath>
#include <map>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <pluginlib/class_list_macros.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace brl_lab_rviz_panel {

namespace {
QDoubleSpinBox *coordinate() {
    auto *spin = new QDoubleSpinBox;
    spin->setRange(-5.0, 5.0);
    spin->setDecimals(3);
    spin->setSingleStep(0.01);
    spin->setSuffix(" m");
    return spin;
}
QString yesNo(bool value) { return value ? "yes" : "<b style='color:#c62828'>no</b>"; }

// F/T calibration poses [deg] (pan, lift, elbow, wrist1, wrist2, wrist3). Shoulder fixed at
// pan 60 (lab; field overrides via pan_deg) / lift -90 / elbow -90; only the wrist changes the tool
// direction. Chosen from FK/collision checks: gravity in the sensor frame covers all axes
// (fit condition ~1.2), tool0 0.49-0.73 m above the base, max 90 deg joint move per step.
struct CalibrationPose { const char *label; const char *description; double joints[6]; };
const CalibrationPose kPoses[] = {
    {"C1", "툴 아래", {60, -90, -90, -90, 90, 0}},
    {"C2", "툴 수평 · 툴축 0°", {60, -90, -90, -90, 0, 0}},
    {"C3", "툴 수평 · 툴축 +45°", {60, -90, -90, -45, 0, 0}},
    {"C4", "툴 수평 · 툴축 +90°", {60, -90, -90, 0, 0, 0}},
    {"C5", "툴 수평 · 툴축 +135°", {60, -90, -90, 45, 0, 0}},
    {"C6", "툴 수평 · 툴축 +90°, 손목3 +90°", {60, -90, -90, 0, 0, 90}},
    {"C7", "툴 위로 45°", {60, -90, -90, -45, -90, 0}},
    {"C8", "툴 위", {60, -90, -90, -90, -90, 0}},
    {"C9", "툴 수평 · 툴축 -45°", {60, -90, -90, -135, 0, 0}},
    {"C10", "툴 수평 · 툴축 0°, 손목3 -90°", {60, -90, -90, -90, 0, -90}},
    // Wrist_3 sweep with the tool down: gravity stays fixed in the sensor frame, so these
    // isolate the wrist_3-angle dependent F/T offset (brl_lab/ft_w3_model.py).
    {"S1", "툴 아래 · 손목3 -180°", {60, -90, -90, -90, 90, -180}},
    {"S2", "툴 아래 · 손목3 -135°", {60, -90, -90, -90, 90, -135}},
    {"S3", "툴 아래 · 손목3 -90°", {60, -90, -90, -90, 90, -90}},
    {"S4", "툴 아래 · 손목3 -45°", {60, -90, -90, -90, 90, -45}},
    {"S5", "툴 아래 · 손목3 0°", {60, -90, -90, -90, 90, 0}},
    {"S6", "툴 아래 · 손목3 +45°", {60, -90, -90, -90, 90, 45}},
    {"S7", "툴 아래 · 손목3 +90°", {60, -90, -90, -90, 90, 90}},
    {"S8", "툴 아래 · 손목3 +135°", {60, -90, -90, -90, 90, 135}},
    {"S9", "툴 아래 · 손목3 +180°", {60, -90, -90, -90, 90, 180}},
};

// Pan of every calibration pose [deg]. Pan turns about the vertical axis, so gravity in the
// sensor frame is unchanged; the field launch faces the arm forward with pan_deg 180.
// Shared with the server through /brl_lab_ft_calibration/pan_deg (default 60, lab).
double calibrationJointDeg(int pose, int joint) {
    if (joint == 0) {
        double pan = kPoses[pose].joints[0];
        ros::param::get("/brl_lab_ft_calibration/pan_deg", pan);
        return pan;
    }
    return kPoses[pose].joints[joint];
}
}  // namespace

BrlActionPanel::BrlActionPanel(QWidget *parent) : rviz::Panel(parent) {
    nh_.setCallbackQueue(&queue_);
    // Contents live in a scroll area so the panel never forces a minimum
    // height on the RViz main window (which blocks resizing/maximizing).
    auto *outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    auto *scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto *content = new QWidget;
    scroll->setWidget(content);
    outer->addWidget(scroll);
    auto *layout = new QVBoxLayout(content);

    // Connection settings
    auto *settings = new QGroupBox("Connection");
    auto *form = new QFormLayout(settings);
    namespace_edit_ = new QLineEdit("/brl_robot");
    robot_edit_ = new QLineEdit("brl_robot");
    status_topic_edit_ = new QLineEdit("/moveit_optimal_controller/status");
    auto *connect_button = new QPushButton("Reconnect");
    form->addRow("Action namespace", namespace_edit_);
    form->addRow("Robot name", robot_edit_);
    form->addRow("HQP status topic", status_topic_edit_);
    form->addRow(connect_button);
    servers_label_ = new QLabel;
    servers_label_->setWordWrap(true);
    form->addRow("Servers", servers_label_);
    layout->addWidget(settings);

    // Arm motion (works on the lab robot)
    auto *arm = new QGroupBox("Arm");
    auto *arm_layout = new QVBoxLayout(arm);
    // First move from an arbitrary lab pose to a BRL working pose. BRL's own planner rejects
    // long plans (point limits), so this panel plans it with move_group and sends it to HQP.
    auto *start_row = new QHBoxLayout;
    start_pose_edit_ = new QLineEdit("90, -33, -140, -33, 87, -88");
    start_pose_edit_->setToolTip("Joint goal in degrees (shoulder_pan ... wrist_3). Default: BRL prepare_navigation step 1.");
    start_pose_button_ = new QPushButton("Go to BRL start pose");
    start_row->addWidget(start_pose_edit_);
    start_row->addWidget(start_pose_button_);
    arm_layout->addWidget(new QLabel("1) BRL start pose [deg] (plan preview in RViz, then confirm):"));
    arm_layout->addLayout(start_row);
    prepare_button_ = new QPushButton("Prepare Navigation");
    arm_layout->addWidget(new QLabel("2) BRL actions:"));
    arm_layout->addWidget(prepare_button_);
    confirm_check_ = new QCheckBox("Confirm before sending (robot will move)");
    confirm_check_->setChecked(true);
    arm_layout->addWidget(confirm_check_);
    layout->addWidget(arm);

    // Tomato tasks (need BRL gripper/camera/frames)
    auto *tomato = new QGroupBox("Tomato tasks (BRL robot: gripper/camera required)");
    auto *tomato_layout = new QFormLayout(tomato);
    tomato_name_edit_ = new QLineEdit("tomato_0");
    tomato_layout->addRow("Tomato name", tomato_name_edit_);
    auto *xyz = new QHBoxLayout;
    x_spin_ = coordinate(); y_spin_ = coordinate(); z_spin_ = coordinate();
    // [BRL_lab] default tomato = gripper tip (graps_link) measured in robot_base_footprint
    // on 2026-11-09 (tf_echo robot_base_footprint graps_link). Pre-pick goes +0.15 m in y.
    // Note: this is in front (+x); its pre-pick IK branches are far from base-pick (>= 96 deg).
    // A tomato on the robot's right, e.g. (0.08, -0.55, 1.10), moves only lift/wrist_1 ~37 deg.
    x_spin_->setValue(0.771); y_spin_->setValue(-0.133); z_spin_->setValue(0.994);
    xyz->addWidget(x_spin_); xyz->addWidget(y_spin_); xyz->addWidget(z_spin_);
    tomato_layout->addRow("Position x y z", xyz);
    pick_button_ = new QPushButton("Pick Tomato");
    tomato_layout->addRow(pick_button_);
    scan_button_ = new QPushButton("Scan Tomato");
    tomato_layout->addRow(scan_button_);
    class_combo_ = new QComboBox;
    class_combo_->addItem("0 ripe", "0");
    class_combo_->addItem("1 unripe", "1");
    class_combo_->addItem("2 rotten", "2");
    place_button_ = new QPushButton("Place Tomato");
    auto *place_row = new QHBoxLayout;
    place_row->addWidget(class_combo_);
    place_row->addWidget(place_button_);
    tomato_layout->addRow("Class", place_row);
    layout->addWidget(tomato);

    // F/T calibration (admittance must be OFF)
    auto *calibration = new QGroupBox("F/T calibration (HQP admittance OFF)");
    auto *calibration_layout = new QVBoxLayout(calibration);
    calibration_info_ = new QLabel;
    calibration_info_->setWordWrap(true);
    calibration_info_->setTextFormat(Qt::RichText);
    calibration_layout->addWidget(calibration_info_);
    calibration_layout->addWidget(new QLabel(QString::fromUtf8(
        "순서대로: Go (계획·미리보기·확인) → 멈춘 뒤 툴에 아무것도 닿지 않게 → Record (4초).\n"
        "C1~C10(툴 방향) + S1~S9(손목3 회전) 모두 같은 UR 프로그램 실행 중에 기록하세요.")));
    auto *grid = new QGridLayout;
    for (int i = 0; i < kCalibrationPoses; ++i) {
        calibration_mark_[i] = new QLabel("○");
        calibration_go_[i] = new QPushButton(QString("Go %1").arg(kPoses[i].label));
        calibration_record_[i] = new QPushButton("Record");
        auto *text = new QLabel(QString::fromUtf8(kPoses[i].description));
        text->setToolTip(QString("%1, %2, %3, %4, %5, %6 deg").arg(calibrationJointDeg(i, 0)).arg(kPoses[i].joints[1])
                             .arg(kPoses[i].joints[2]).arg(kPoses[i].joints[3]).arg(kPoses[i].joints[4]).arg(kPoses[i].joints[5]));
        grid->addWidget(calibration_mark_[i], i, 0);
        grid->addWidget(calibration_go_[i], i, 1);
        grid->addWidget(text, i, 2);
        grid->addWidget(calibration_record_[i], i, 3);
        connect(calibration_go_[i], &QPushButton::clicked, this, [this, i]() { goCalibrationPose(i); });
        connect(calibration_record_[i], &QPushButton::clicked, this, [this, i]() { recordCalibrationPose(i); });
    }
    calibration_layout->addLayout(grid);
    auto *calibration_buttons = new QHBoxLayout;
    auto *new_dataset_button = new QPushButton("New dataset");
    auto *fit_button = new QPushButton("Fit model");
    auto_button_ = new QPushButton(QString::fromUtf8("Auto: 19개 자세 이동 + 기록 + Fit"));
    auto_button_->setStyleSheet("QPushButton { font-weight: bold; }");
    calibration_layout->addWidget(auto_button_);
    calibration_buttons->addWidget(new_dataset_button);
    calibration_buttons->addWidget(fit_button);
    calibration_layout->addLayout(calibration_buttons);
    calibration_result_ = new QLabel;
    calibration_result_->setWordWrap(true);
    calibration_result_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    calibration_layout->addWidget(calibration_result_);
    connect(new_dataset_button, SIGNAL(clicked()), this, SLOT(newCalibrationDataset()));
    connect(fit_button, SIGNAL(clicked()), this, SLOT(fitCalibration()));
    connect(auto_button_, SIGNAL(clicked()), this, SLOT(startAutoCalibration()));
    layout->addWidget(calibration);

    // Goal progress
    auto *goal = new QGroupBox("Goal");
    auto *goal_layout = new QVBoxLayout(goal);
    active_label_ = new QLabel("Idle");
    progress_bar_ = new QProgressBar;
    progress_bar_->setRange(0, 100);
    feedback_label_ = new QLabel;
    feedback_label_->setWordWrap(true);
    result_label_ = new QLabel;
    result_label_->setWordWrap(true);
    cancel_button_ = new QPushButton("Cancel (hold position)");
    cancel_button_->setStyleSheet("QPushButton { background-color: #c62828; color: white; font-weight: bold; }");
    goal_layout->addWidget(active_label_);
    goal_layout->addWidget(progress_bar_);
    goal_layout->addWidget(feedback_label_);
    goal_layout->addWidget(result_label_);
    goal_layout->addWidget(cancel_button_);
    layout->addWidget(goal);

    // HQP controller status
    auto *hqp = new QGroupBox("HQP controller");
    auto *hqp_layout = new QVBoxLayout(hqp);
    hqp_label_ = new QLabel("No status yet");
    hqp_label_->setWordWrap(true);
    hqp_label_->setTextFormat(Qt::RichText);
    hqp_layout->addWidget(hqp_label_);
    layout->addWidget(hqp);
    layout->addStretch();

    connect(connect_button, SIGNAL(clicked()), this, SLOT(connectClients()));
    connect(prepare_button_, SIGNAL(clicked()), this, SLOT(sendPrepareNavigation()));
    connect(scan_button_, SIGNAL(clicked()), this, SLOT(sendScan()));
    connect(pick_button_, SIGNAL(clicked()), this, SLOT(sendPick()));
    connect(place_button_, SIGNAL(clicked()), this, SLOT(sendPlace()));
    connect(cancel_button_, SIGNAL(clicked()), this, SLOT(cancelGoal()));
    connect(start_pose_button_, SIGNAL(clicked()), this, SLOT(goToStartPose()));
    timer_ = new QTimer(this);
    connect(timer_, SIGNAL(timeout()), this, SLOT(refresh()));
}

void BrlActionPanel::onInitialize() {
    spinner_.reset(new ros::AsyncSpinner(1, &queue_));
    spinner_->start();
    connectClients();
    timer_->start(100);
}

void BrlActionPanel::connectClients() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_.empty()) {
            QMessageBox::warning(this, "BRL actions", "Cancel the running goal before reconnecting.");
            return;
        }
        have_status_ = false;
    }
    std::string ns = namespace_edit_->text().trimmed().toStdString();
    while (ns.size() > 1 && ns.back() == '/') ns.pop_back();
    // Stop callbacks while clients are replaced, then resume.
    if (spinner_) spinner_->stop();
    prepare_.reset(new Client<brl_interfaces::PrepareNavigationAction>(nh_, ns + "/prepare_navigation", false));
    scan_.reset(new Client<brl_interfaces::ScanTomatoAction>(nh_, ns + "/scan_tomato", false));
    pick_.reset(new Client<brl_interfaces::PickTomatoAction>(nh_, ns + "/pick_tomato", false));
    place_.reset(new Client<brl_interfaces::PlaceTomatoAction>(nh_, ns + "/place_tomato", false));
    status_sub_ = nh_.subscribe(status_topic_edit_->text().trimmed().toStdString(), 1,
                                &BrlActionPanel::statusCallback, this);
    joint_sub_ = nh_.subscribe("/joint_states", 1, &BrlActionPanel::jointCallback, this);
    progress_sub_ = nh_.subscribe("/brl_lab_ft_calibration/progress", 10, &BrlActionPanel::progressCallback, this);
    plan_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/rci_action_manager/moveit_plan", 1);
    preview_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/brl_lab/start_pose_preview", 1, true);
    if (spinner_) spinner_->start();
    ROS_INFO_STREAM("[BRL panel] Action clients on " << ns << "/{prepare_navigation,scan_tomato,pick_tomato,place_tomato}");
}

bool BrlActionPanel::busy() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !active_.empty() || auto_running_;
}

bool BrlActionPanel::confirm(const QString &name) {
    if (busy()) {
        QMessageBox::warning(this, "BRL actions", "A goal is already running. Cancel it first.");
        return false;
    }
    if (!confirm_check_->isChecked()) return true;
    return QMessageBox::question(this, "Send " + name,
                                 "Send " + name + "?\nThe robot will move through HQP.",
                                 QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes;
}

template <class ClientT, class Goal, class Feedback, class Result>
void BrlActionPanel::send(ClientT &client, const Goal &goal, const std::string &name) {
    if (!client.isServerConnected()) {
        ROS_WARN_STREAM("[BRL panel] " << name << " not sent: action server not connected");
        QMessageBox::warning(this, "BRL actions",
                             QString::fromStdString(name + " server is not connected. Is the BRL motion planner running?"));
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_ = name;
        progress_ = 0;
        feedback_.clear();
        result_.clear();
        cancel_ = [&client]() { client.cancelGoal(); };
    }
    ROS_INFO_STREAM("[BRL panel] Sending " << name);
    client.sendGoal(
        goal,
        [this, name](const actionlib::SimpleClientGoalState &state, const typename Result::ConstPtr &result) {
            std::lock_guard<std::mutex> lock(mutex_);
            result_ = name + ": " + state.toString() + (result ? " - " + result->message : std::string());
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) progress_ = 100;
            active_.clear();
            cancel_ = nullptr;
        },
        []() {},
        [this](const typename Feedback::ConstPtr &feedback) {
            std::lock_guard<std::mutex> lock(mutex_);
            progress_ = feedback->progress;
            feedback_ = feedback->status;
        });
}

void BrlActionPanel::sendPrepareNavigation() {
    if (!confirm("Prepare Navigation")) return;
    brl_interfaces::PrepareNavigationGoal goal;
    goal.robot_name = robot_edit_->text().toStdString();
    send<Client<brl_interfaces::PrepareNavigationAction>, brl_interfaces::PrepareNavigationGoal,
         brl_interfaces::PrepareNavigationFeedback, brl_interfaces::PrepareNavigationResult>(*prepare_, goal, "Prepare Navigation");
}

void BrlActionPanel::sendScan() {
    if (!confirm("Scan Tomato")) return;
    brl_interfaces::ScanTomatoGoal goal;
    goal.robot_name = robot_edit_->text().toStdString();
    send<Client<brl_interfaces::ScanTomatoAction>, brl_interfaces::ScanTomatoGoal,
         brl_interfaces::ScanTomatoFeedback, brl_interfaces::ScanTomatoResult>(*scan_, goal, "Scan Tomato");
}

void BrlActionPanel::sendPick() {
    if (!confirm("Pick Tomato")) return;
    brl_interfaces::PickTomatoGoal goal;
    goal.robot_name = robot_edit_->text().toStdString();
    goal.tomato_name = tomato_name_edit_->text().toStdString();
    goal.tomato_position.x = x_spin_->value();
    goal.tomato_position.y = y_spin_->value();
    goal.tomato_position.z = z_spin_->value();
    send<Client<brl_interfaces::PickTomatoAction>, brl_interfaces::PickTomatoGoal,
         brl_interfaces::PickTomatoFeedback, brl_interfaces::PickTomatoResult>(*pick_, goal, "Pick Tomato");
}

void BrlActionPanel::sendPlace() {
    if (!confirm("Place Tomato")) return;
    brl_interfaces::PlaceTomatoGoal goal;
    goal.robot_name = robot_edit_->text().toStdString();
    goal.tomato_name = tomato_name_edit_->text().toStdString();
    goal.tomato_class = class_combo_->currentData().toString().toStdString();
    send<Client<brl_interfaces::PlaceTomatoAction>, brl_interfaces::PlaceTomatoGoal,
         brl_interfaces::PlaceTomatoFeedback, brl_interfaces::PlaceTomatoResult>(*place_, goal, "Place Tomato");
}

void BrlActionPanel::cancelGoal() {
    std::function<void()> cancel;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (hqp_goal_) {
            feedback_ = "Cancel requested; holding the current position";
            hqp_goal_ = false;
            active_.clear();
            result_ = hqp_goal_name_ + ": CANCELED (hold sent)";
        }
        cancel = cancel_;
        if (cancel) feedback_ = "Cancel requested; planner holds the current position";
    }
    bool automatic = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        automatic = auto_running_;
    }
    if (automatic) {
        ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("/brl_lab_ft_calibration/stop");
        std_srvs::Trigger srv;
        if (!client.call(srv)) sendHold();
        return;
    }
    if (cancel) cancel();
    else sendHold();
}

void BrlActionPanel::jointCallback(const sensor_msgs::JointState::ConstPtr &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    joints_ = *message;
    have_joints_ = true;
}

bool BrlActionPanel::hqpReady(std::string &why) const {
    // Same readiness rule as send_rviz_moveit_plan.py (manual mode).
    if (!have_status_) { why = "no HQP status"; return false; }
    const double age = (ros::Time::now() - status_stamp_).toSec();
    const bool reason_ok = status_.reason.empty() || status_.reason == "goal tolerance satisfied" ||
                           status_.reason == "replaced by a newer trajectory" ||
                           (status_.last_result == "ABORTED" && status_.reason == "goal tolerance timeout");
    if (age > 0.5) why = "HQP status stale";
    else if (status_.state != "HOLDING") why = "HQP state is " + status_.state + " (must be HOLDING)";
    else if (!reason_ok) why = "HQP reason: " + status_.reason;
    else if (!status_.command_enabled) why = "HQP command disabled";
    else if (!status_.ft_bias_ready) why = "F/T bias not ready";
    else return true;
    return false;
}

void BrlActionPanel::sendHold() {
    // Replace the active HQP trajectory with a hold at the measured position (as BRL's _hqp_hold).
    std::vector<std::string> names;
    if (!nh_.getParam("/moveit_optimal_controller/joint_names", names) || names.size() != 6) return;
    trajectory_msgs::JointTrajectory hold;
    hold.joint_names = names;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!have_status_) return;
        for (double t : {0.0, 0.5}) {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.assign(status_.actual.begin(), status_.actual.end());
            point.velocities.assign(6, 0.0);
            point.accelerations.assign(6, 0.0);
            point.time_from_start = ros::Duration(t);
            hold.points.push_back(point);
        }
    }
    plan_pub_.publish(hold);
    ROS_INFO("[BRL panel] Hold trajectory sent to HQP");
}

void BrlActionPanel::goToStartPose() {
    if (busy()) {
        QMessageBox::warning(this, "BRL start pose", "A goal is already running. Cancel it first.");
        return;
    }
    std::vector<std::string> names;
    if (!nh_.getParam("/moveit_optimal_controller/joint_names", names) || names.size() != 6) {
        QMessageBox::warning(this, "BRL start pose", "No /moveit_optimal_controller/joint_names: is the HQP controller running?");
        return;
    }
    const QStringList fields = start_pose_edit_->text().split(',', QString::SkipEmptyParts);
    std::vector<double> goal;
    for (const QString &field : fields) {
        bool ok = false;
        const double degrees = field.trimmed().toDouble(&ok);
        if (!ok) break;
        goal.push_back(degrees * M_PI / 180.0);
    }
    if (goal.size() != 6) {
        QMessageBox::warning(this, "BRL start pose", "Enter six joint angles in degrees, separated by commas.");
        return;
    }
    moveToJoints(goal, "Go to BRL start pose");
}

void BrlActionPanel::moveToJoints(const std::vector<double> &goal, const std::string &name) {
    const QString title = QString::fromStdString(name);
    if (busy()) {
        QMessageBox::warning(this, title, "A goal is already running. Cancel it first.");
        return;
    }
    std::vector<std::string> names;
    if (!nh_.getParam("/moveit_optimal_controller/joint_names", names) || names.size() != 6) {
        QMessageBox::warning(this, title, "No /moveit_optimal_controller/joint_names: is the HQP controller running?");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string why;
        if (!hqpReady(why)) {
            QMessageBox::warning(this, title, QString::fromStdString("HQP not ready: " + why));
            return;
        }
    }

    // Plan with the same move_group (summit model, collision checked). Start = current state.
    moveit_msgs::GetMotionPlan srv;
    auto &request = srv.request.motion_plan_request;
    request.group_name = "manipulator";
    request.start_state.is_diff = true;
    request.num_planning_attempts = 5;
    request.allowed_planning_time = 5.0;
    moveit_msgs::Constraints constraints;
    for (size_t i = 0; i < 6; ++i) {
        moveit_msgs::JointConstraint joint;
        joint.joint_name = names[i];
        joint.position = goal[i];
        joint.tolerance_above = joint.tolerance_below = 0.001;
        joint.weight = 1.0;
        constraints.joint_constraints.push_back(joint);
    }
    request.goal_constraints.push_back(constraints);
    ros::ServiceClient planner = nh_.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path");
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const bool called = planner.waitForExistence(ros::Duration(2.0)) && planner.call(srv);
    QApplication::restoreOverrideCursor();
    if (!called || srv.response.motion_plan_response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        QMessageBox::warning(this, title,
                             QString("Planning failed (move_group error %1). Check RViz/move_group.")
                                 .arg(called ? srv.response.motion_plan_response.error_code.val : 0));
        return;
    }
    trajectory_msgs::JointTrajectory trajectory = srv.response.motion_plan_response.trajectory.joint_trajectory;
    if (trajectory.points.size() < 2 || trajectory.joint_names.size() != 6) {
        QMessageBox::warning(this, title, "Planner returned an empty trajectory.");
        return;
    }

    // Preview in RViz ("Start pose preview" display).
    moveit_msgs::DisplayTrajectory preview;
    preview.trajectory_start = srv.response.motion_plan_response.trajectory_start;
    preview.trajectory.push_back(srv.response.motion_plan_response.trajectory);
    preview_pub_.publish(preview);

    // Summary: per-joint travel along the path.
    QStringList travel;
    for (size_t j = 0; j < 6; ++j) {
        double sum = 0;
        for (size_t k = 1; k < trajectory.points.size(); ++k)
            sum += std::abs(trajectory.points[k].positions[j] - trajectory.points[k - 1].positions[j]);
        travel << QString("%1 %2").arg(QString::fromStdString(trajectory.joint_names[j]).replace("_joint", ""))
                      .arg(sum * 180.0 / M_PI, 0, 'f', 0);
    }
    const QString summary = QString("Planned path: %1 points, MoveIt time %2 s (HQP retimes it).\n"
                                    "Joint travel [deg]: %3\n\n"
                                    "Check the 'Start pose preview' path in RViz.\n"
                                    "Send it to HQP? The robot will move.")
                                .arg(trajectory.points.size())
                                .arg(trajectory.points.back().time_from_start.toSec(), 0, 'f', 1)
                                .arg(travel.join(", "));
    if (QMessageBox::question(this, title, summary, QMessageBox::Yes | QMessageBox::No,
                              QMessageBox::No) != QMessageBox::Yes)
        return;

    // Re-check right before sending: HQP ready and the robot still at the plan start.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string why;
        if (!hqpReady(why)) {
            QMessageBox::warning(this, title, QString::fromStdString("HQP not ready: " + why));
            return;
        }
        if (!have_joints_) {
            QMessageBox::warning(this, title, "No /joint_states.");
            return;
        }
        std::map<std::string, double> current;
        for (size_t i = 0; i < joints_.name.size() && i < joints_.position.size(); ++i) current[joints_.name[i]] = joints_.position[i];
        double gap = 0;
        for (size_t j = 0; j < 6; ++j) {
            auto found = current.find(trajectory.joint_names[j]);
            if (found == current.end()) { gap = 1e9; break; }
            gap = std::max(gap, std::abs(found->second - trajectory.points.front().positions[j]));
        }
        double tolerance = 0.05;
        nh_.param("/moveit_optimal_controller/start_tolerance", tolerance, tolerance);
        if (gap > tolerance) {
            QMessageBox::warning(this, title,
                                 QString("Robot moved since planning (gap %1 deg). Plan again.").arg(gap * 180.0 / M_PI, 0, 'f', 1));
            return;
        }
        if (plan_pub_.getNumSubscribers() == 0) {
            QMessageBox::warning(this, title, "No HQP trajectory subscriber.");
            return;
        }
        hqp_before_id_ = std::max(status_.trajectory_id, status_.result_trajectory_id);
        hqp_goal_ = true;
        hqp_sent_ = ros::Time::now();
        active_ = name;
        hqp_goal_name_ = name;
        progress_ = 0;
        feedback_ = "Sent to HQP";
        result_.clear();
        cancel_ = nullptr;
    }
    trajectory.header.stamp = ros::Time(0);  // start at the next HQP tick
    plan_pub_.publish(trajectory);
    ROS_INFO_STREAM("[BRL panel] " << name << " trajectory sent to HQP (" << trajectory.points.size() << " points)");
}

bool BrlActionPanel::admittanceOff(QString &why) {
    bool admittance = true;
    if (!nh_.getParam("/moveit_optimal_controller/admittance_enabled", admittance)) {
        why = "HQP controller is not running.";
        return false;
    }
    if (admittance) {
        why = "Admittance is ON. Restart HQP:\nroslaunch summit_ur5_hardware_setup ur5e_moveit_brl_lab.launch admittance_enabled:=false";
        return false;
    }
    return true;
}

void BrlActionPanel::goCalibrationPose(int index) {
    QString why;
    if (!admittanceOff(why)) {
        QMessageBox::warning(this, "F/T calibration", why);
        return;
    }
    std::vector<double> goal;
    for (int j = 0; j < 6; ++j) goal.push_back(calibrationJointDeg(index, j) * M_PI / 180.0);
    moveToJoints(goal, std::string("Calibration ") + kPoses[index].label);
}

void BrlActionPanel::recordCalibrationPose(int index) {
    QString why;
    if (!admittanceOff(why)) {
        QMessageBox::warning(this, "F/T calibration", why);
        return;
    }
    // Refuse unless the robot is at this pose (within 1 deg).
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::string> names;
        nh_.getParam("/moveit_optimal_controller/joint_names", names);
        std::map<std::string, double> current;
        for (size_t i = 0; i < joints_.name.size() && i < joints_.position.size(); ++i) current[joints_.name[i]] = joints_.position[i];
        double gap = 0;
        for (size_t j = 0; j < names.size() && j < 6; ++j) {
            auto found = current.find(names[j]);
            gap = std::max(gap, found == current.end() ? 1e9 : std::abs(found->second - calibrationJointDeg(index, static_cast<int>(j)) * M_PI / 180.0));
        }
        if (names.size() != 6 || gap > M_PI / 180.0) {
            QMessageBox::warning(this, "F/T calibration", QString("Robot is not at %1 (gap %2 deg). Press Go %1 first.")
                                                              .arg(kPoses[index].label).arg(gap * 180.0 / M_PI, 0, 'f', 1));
            return;
        }
    }
    nh_.setParam("/brl_lab_ft_calibration/label", std::string(kPoses[index].label));
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("/brl_lab_ft_calibration/capture");
    std_srvs::Trigger srv;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const bool called = client.waitForExistence(ros::Duration(2.0)) && client.call(srv);
    QApplication::restoreOverrideCursor();
    if (!called) {
        calibration_result_->setText("Capture service not available (brl_lab_ft_calibration node).");
        return;
    }
    calibration_result_->setText(QString::fromStdString(srv.response.message));
    if (srv.response.success) calibration_mark_[index]->setText("●");
}

void BrlActionPanel::newCalibrationDataset() {
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("/brl_lab_ft_calibration/new_dataset");
    std_srvs::Trigger srv;
    if (!(client.waitForExistence(ros::Duration(2.0)) && client.call(srv))) {
        calibration_result_->setText("Calibration service not available.");
        return;
    }
    for (auto *mark : calibration_mark_) mark->setText("○");
    calibration_result_->setText(QString::fromStdString(srv.response.message));
}

void BrlActionPanel::fitCalibration() {
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("/brl_lab_ft_calibration/fit");
    std_srvs::Trigger srv;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const bool called = client.waitForExistence(ros::Duration(2.0)) && client.call(srv);
    QApplication::restoreOverrideCursor();
    calibration_result_->setText(called ? QString::fromStdString(srv.response.message) : "Calibration service not available.");
}

void BrlActionPanel::progressCallback(const std_msgs::String::ConstPtr &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto_progress_ = message->data;
    auto_running_ = message->data.rfind("RUNNING", 0) == 0;
    auto_progress_new_ = true;
}

void BrlActionPanel::startAutoCalibration() {
    QString why;
    if (!admittanceOff(why)) {
        QMessageBox::warning(this, "F/T calibration", why);
        return;
    }
    if (busy()) {
        QMessageBox::warning(this, "F/T calibration", "A goal is already running. Cancel it first.");
        return;
    }
    const QString text = QString::fromUtf8(
        "F/T 자동 보정을 시작할까요?\n\n"
        "- 새 데이터셋으로 C1~C10, S1~S9 (19개) 자세를 차례로 이동·기록하고 마지막에 Fit합니다.\n"
        "- 각 이동은 move_group으로 계획(충돌 검사)한 뒤 HQP로 실행합니다. 약 5~10분 걸립니다.\n"
        "- 어깨 회전 60° 방향, 팔 받침 위 0.5~0.75 m 주변을 비워 두고 툴에 아무것도 닿지 않게 하세요.\n"
        "- 진행 중 UR 프로그램을 멈추지 마세요. 멈추려면 빨간 Cancel 버튼 (현재 위치 유지).\n\n"
        "로봇이 움직입니다.");
    if (QMessageBox::question(this, "F/T auto calibration", text, QMessageBox::Yes | QMessageBox::No,
                              QMessageBox::No) != QMessageBox::Yes)
        return;
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>("/brl_lab_ft_calibration/run_all");
    std_srvs::Trigger srv;
    if (!(client.waitForExistence(ros::Duration(2.0)) && client.call(srv))) {
        calibration_result_->setText("Calibration service not available (brl_lab_ft_calibration node).");
        return;
    }
    calibration_result_->setText(QString::fromStdString(srv.response.message));
    if (srv.response.success) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto_running_ = true;
        result_.clear();
    }
    for (auto *mark : calibration_mark_) mark->setText("○");
}

void BrlActionPanel::statusCallback(const rci_action_manager::MoveitOptimalStatus::ConstPtr &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = *message;
    status_stamp_ = ros::Time::now();
    have_status_ = true;
}

void BrlActionPanel::refresh() {
    auto mark = [](const char *name, bool connected) {
        return QString("%1 %2").arg(connected ? "&#9679;" : "&#9675;").arg(name);
    };
    servers_label_->setTextFormat(Qt::RichText);
    servers_label_->setText(QStringList{
        mark("prepare", prepare_ && prepare_->isServerConnected()),
        mark("scan", scan_ && scan_->isServerConnected()),
        mark("pick", pick_ && pick_->isServerConnected()),
        mark("place", place_ && place_->isServerConnected())}.join("&nbsp;&nbsp;"));

    std::lock_guard<std::mutex> lock(mutex_);
    if (hqp_goal_ && have_status_) {
        if (status_.result_trajectory_id > hqp_before_id_ && status_.last_result != "NONE") {
            result_ = hqp_goal_name_ + ": " + status_.last_result + " - " + status_.reason;
            if (status_.last_result == "SUCCEEDED") progress_ = 100;
            hqp_goal_ = false;
            active_.clear();
        } else if (status_.trajectory_id <= hqp_before_id_ && (ros::Time::now() - hqp_sent_).toSec() > 3.0) {
            result_ = hqp_goal_name_ + ": not accepted by HQP (" + status_.reason + ")";
            hqp_goal_ = false;
            active_.clear();
        } else if (status_.trajectory_id > hqp_before_id_) {
            feedback_ = "HQP " + status_.state + ", elapsed " + std::to_string(static_cast<int>(status_.elapsed)) + " s";
        }
    }
    const bool idle = active_.empty() && !auto_running_;
    active_label_->setText(auto_running_ ? QString::fromUtf8("Running: F/T 자동 보정")
                           : idle ? "Idle" : QString::fromStdString("Running: " + active_));
    if (auto_progress_new_) {
        for (int i = 0; i < kCalibrationPoses; ++i) {
            const std::string tag = std::string(" ") + kPoses[i].label + ": " + kPoses[i].label + ":";
            if (auto_progress_.find(tag) != std::string::npos) calibration_mark_[i]->setText("●");
        }
        calibration_result_->setText(QString::fromStdString(auto_progress_));
        feedback_label_->setText(QString::fromStdString(auto_progress_.substr(0, 120)));
        auto_progress_new_ = false;
    }
    if (auto_button_) auto_button_->setEnabled(idle);
    progress_bar_->setValue(progress_);
    feedback_label_->setText(QString::fromStdString(feedback_));
    result_label_->setText(QString::fromStdString(result_));
    for (auto *button : {prepare_button_, scan_button_, pick_button_, place_button_, start_pose_button_}) button->setEnabled(idle);
    for (int i = 0; i < kCalibrationPoses; ++i) {
        calibration_go_[i]->setEnabled(idle);
        calibration_record_[i]->setEnabled(idle);
    }
    bool admittance = true;
    const bool known = nh_.getParam("/moveit_optimal_controller/admittance_enabled", admittance);
    calibration_info_->setText(!known ? "<b style='color:#c62828'>HQP not running</b>"
                               : admittance ? "<b style='color:#c62828'>Admittance ON</b> - restart HQP with admittance_enabled:=false"
                                            : "Admittance OFF - ready");
    cancel_button_->setEnabled(!idle);

    if (!have_status_) {
        hqp_label_->setText("<b style='color:#c62828'>No HQP status</b> - is ur5e_moveit_brl_lab.launch running?");
        return;
    }
    const double age = (ros::Time::now() - status_stamp_).toSec();
    QString text = QString("State: <b>%1</b>%2<br>Last result: %3 &nbsp; %4<br>"
                           "Command enabled: %5 &nbsp; F/T bias ready: %6<br>"
                           "Tracking RMS %7 / max %8 rad")
        .arg(QString::fromStdString(status_.state))
        .arg(age > 0.5 ? QString(" <b style='color:#c62828'>(stale %1 s)</b>").arg(age, 0, 'f', 1) : QString())
        .arg(QString::fromStdString(status_.last_result))
        .arg(QString::fromStdString(status_.reason).toHtmlEscaped())
        .arg(yesNo(status_.command_enabled))
        .arg(yesNo(status_.ft_bias_ready))
        .arg(status_.tracking_rms, 0, 'f', 4)
        .arg(status_.tracking_max, 0, 'f', 4);
    hqp_label_->setText(text);
}

void BrlActionPanel::save(rviz::Config config) const {
    rviz::Panel::save(config);
    config.mapSetValue("ActionNamespace", namespace_edit_->text());
    config.mapSetValue("RobotName", robot_edit_->text());
    config.mapSetValue("StatusTopic", status_topic_edit_->text());
    config.mapSetValue("ConfirmBeforeSend", confirm_check_->isChecked());
    config.mapSetValue("StartPoseDeg", start_pose_edit_->text());
}

void BrlActionPanel::load(const rviz::Config &config) {
    rviz::Panel::load(config);
    QString value;
    if (config.mapGetString("ActionNamespace", &value)) namespace_edit_->setText(value);
    if (config.mapGetString("RobotName", &value)) robot_edit_->setText(value);
    if (config.mapGetString("StatusTopic", &value)) status_topic_edit_->setText(value);
    if (config.mapGetString("StartPoseDeg", &value)) start_pose_edit_->setText(value);
    bool confirm_send = true;
    if (config.mapGetBool("ConfirmBeforeSend", &confirm_send)) confirm_check_->setChecked(confirm_send);
    connectClients();
}

}  // namespace brl_lab_rviz_panel

PLUGINLIB_EXPORT_CLASS(brl_lab_rviz_panel::BrlActionPanel, rviz::Panel)

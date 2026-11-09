#pragma once
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <rviz/panel.h>
#include <actionlib/client/simple_action_client.h>
#include <brl_interfaces/PrepareNavigationAction.h>
#include <brl_interfaces/ScanTomatoAction.h>
#include <brl_interfaces/PickTomatoAction.h>
#include <brl_interfaces/PlaceTomatoAction.h>
#include <rci_action_manager/MoveitOptimalStatus.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <vector>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#endif

class QLabel;
class QLineEdit;
class QPushButton;
class QProgressBar;
class QDoubleSpinBox;
class QComboBox;
class QCheckBox;
class QTimer;

namespace brl_lab_rviz_panel {

// Sends BRL motion planner actions and shows the HQP controller status.
// Only one goal is active at a time; ROS callbacks only store data, and a
// Qt timer copies it to the widgets.
class BrlActionPanel : public rviz::Panel {
    Q_OBJECT
public:
    explicit BrlActionPanel(QWidget *parent = nullptr);
    void onInitialize() override;
    void save(rviz::Config config) const override;
    void load(const rviz::Config &config) override;

protected Q_SLOTS:
    void connectClients();
    void sendPrepareNavigation();
    void sendScan();
    void sendPick();
    void sendPlace();
    void cancelGoal();
    void refresh();
    void goToStartPose();
    void newCalibrationDataset();
    void fitCalibration();
    void startAutoCalibration();

private:
    template <class Action> using Client = actionlib::SimpleActionClient<Action>;
    bool confirm(const QString &name);
    bool busy() const;
    template <class Client, class Goal, class Feedback, class Result>
    void send(Client &client, const Goal &goal, const std::string &name);
    void statusCallback(const rci_action_manager::MoveitOptimalStatus::ConstPtr &message);
    void jointCallback(const sensor_msgs::JointState::ConstPtr &message);
    bool hqpReady(std::string &why) const;  // caller holds mutex_
    void sendHold();                         // replace the HQP trajectory with a hold
    // Plan (move_group, collision checked), preview, confirm, send to HQP. Goal in radians.
    void moveToJoints(const std::vector<double> &goal, const std::string &name);
    void goCalibrationPose(int index);
    void recordCalibrationPose(int index);
    bool admittanceOff(QString &why);
    void progressCallback(const std_msgs::String::ConstPtr &message);

    // Own queue + spinner: does not depend on RViz processing the global queue.
    ros::CallbackQueue queue_;
    ros::NodeHandle nh_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    ros::Subscriber status_sub_, joint_sub_, progress_sub_;
    ros::Publisher plan_pub_, preview_pub_;
    std::unique_ptr<Client<brl_interfaces::PrepareNavigationAction>> prepare_;
    std::unique_ptr<Client<brl_interfaces::ScanTomatoAction>> scan_;
    std::unique_ptr<Client<brl_interfaces::PickTomatoAction>> pick_;
    std::unique_ptr<Client<brl_interfaces::PlaceTomatoAction>> place_;

    mutable std::mutex mutex_;
    std::string active_;            // name of the running goal, empty when idle
    std::function<void()> cancel_;  // cancels the running goal
    int progress_ = 0;
    std::string feedback_, result_;
    rci_action_manager::MoveitOptimalStatus status_;
    ros::Time status_stamp_;
    bool have_status_ = false;
    sensor_msgs::JointState joints_;
    bool have_joints_ = false;
    // "Go to BRL start pose": HQP trajectory sent by this panel (not a BRL action).
    bool hqp_goal_ = false;
    uint64_t hqp_before_id_ = 0;
    ros::Time hqp_sent_;
    std::string hqp_goal_name_;
    // F/T calibration (10 fixed poses; capture/fit by brl_lab/ft_calibration_server.py)
    static constexpr int kCalibrationPoses = 19;  // C1-C10 orientations + S1-S9 wrist_3 sweep
    QLabel *calibration_mark_[kCalibrationPoses] = {};
    QPushButton *calibration_go_[kCalibrationPoses] = {}, *calibration_record_[kCalibrationPoses] = {};
    QLabel *calibration_info_ = nullptr, *calibration_result_ = nullptr;
    QPushButton *auto_button_ = nullptr;
    // Automatic run state (from /brl_lab_ft_calibration/progress)
    bool auto_running_ = false;
    std::string auto_progress_;
    bool auto_progress_new_ = false;

    QLineEdit *namespace_edit_, *robot_edit_, *status_topic_edit_, *tomato_name_edit_, *start_pose_edit_;
    QCheckBox *confirm_check_;
    QLabel *servers_label_, *active_label_, *feedback_label_, *result_label_, *hqp_label_;
    QProgressBar *progress_bar_;
    QPushButton *prepare_button_, *scan_button_, *pick_button_, *place_button_, *cancel_button_, *start_pose_button_;
    QDoubleSpinBox *x_spin_, *y_spin_, *z_spin_;
    QComboBox *class_combo_;
    QTimer *timer_;
};

}  // namespace brl_lab_rviz_panel

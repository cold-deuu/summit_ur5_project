# field_robot: BRL motion planner (field PC)

The planning side of the field test. HQP runs on the other PC (branch `field_comp`).

## Install
```bash
cd ~/catkin_ws/src
git clone -b field_robot https://github.com/cold-deuu/summit_ur5_project.git field_robot
cd .. && catkin_make && source devel/setup.bash
```
Remove the old BRL `motion_planning` and `brl_interfaces` packages from the workspace first. Delete `robotiq_85_msgs` from the clone if the workspace already has it.

| Package | Contents |
|---|---|
| `motion_planning` | BRL planner. It sends plans to HQP instead of executing them in MoveIt. |
| `brl_interfaces` | Adds `VisualServoingAction` |
| `rci_action_manager` | Only the HQP status message (`MoveitOptimalStatus`). The name must match HQP's package. |
| `robotiq_85_msgs` | Gripper messages |

Check: `rosmsg md5 rci_action_manager/MoveitOptimalStatus` gives the same result on both PCs.

## Run (with HQP visual servoing)
```bash
rosrun motion_planning motion_planner_node.py \
  _hqp_visual_servoing:=true _enable_visual_servoing:=false \
  _cartesian_via_hqp:=true _normalize_pose_goals:=true
```
Leave `ignore_max_points` off here: BRL's point limits fit BRL's own MoveIt config.

The HQP servo node (`brl_hqp_servo.py`) runs on the desktop with HQP (`field_comp`, `ur5e_moveit_field.launch`) and serves `/brl_robot/hqp_visual_servoing`.

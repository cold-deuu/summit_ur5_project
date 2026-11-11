#!/bin/bash
# Is this PC ready to run the HQP controller? Prints PASS / WARN / FAIL with the fix for each FAIL.
#   check.sh                  installation only (no ROS master needed)
#   check.sh --live [ns]      also the running robot: driver topics, controller, model (default ns /brl_robot/ur5e)
D=$(cd "$(dirname "$0")" && pwd); FAILS=0; WARNS=0; MODE=$1; NS=${2:-/brl_robot/ur5e}
pass() { printf '[PASS] %s\n' "$1"; }
warn() { printf '[WARN] %s\n       -> %s\n' "$1" "$2"; WARNS=$((WARNS+1)); }
fail() { printf '[FAIL] %s\n       -> %s\n' "$1" "$2"; FAILS=$((FAILS+1)); }
ver() { grep -ho 'PACKAGE_VERSION "[0-9.]*"' $1 2>/dev/null | head -1 | grep -o '[0-9.]*'; }

echo "--- system"
. /etc/os-release
[ "$VERSION_ID" = 18.04 ] && pass "Ubuntu 18.04" || warn "Ubuntu $VERSION_ID" "written for 18.04 + Melodic"
[ "$ROS_DISTRO" = melodic ] && pass "ROS_DISTRO=melodic" || fail "ROS_DISTRO='$ROS_DISTRO'" "source /opt/ros/melodic/setup.bash (and the workspace devel/setup.bash)"

echo "--- libraries (install_deps.sh)"
for x in "fcl 0.6.1 /usr/local/lib/cmake/fcl/fcl-config-version.cmake" \
         "toppra 0.6.4 /usr/local/lib/cmake/toppra/toppraConfigVersion.cmake" \
         "eiquadprog 1.2.9 /usr/local/lib/cmake/eiquadprog/eiquadprogConfigVersion.cmake"; do
  set -- $x; v=$(ver $3)
  [ "$v" = "$2" ] && pass "$1 $v" || fail "$1: found '${v:-none}', need $2" "run install_deps.sh"
done
dpkg -s ros-melodic-eiquadprog >/dev/null 2>&1 && fail "ros-melodic-eiquadprog (1.2.5) installed: link error 'multiple definition of solve_quadprog2'" \
  "sudo apt remove ros-melodic-eiquadprog  (install_deps.sh does this)" || pass "no apt eiquadprog shadowing 1.2.9"
v=$(ver /opt/ros/melodic/share/pinocchio/cmake/pinocchioConfigVersion.cmake)
[ -n "$v" ] || v=$(dpkg -s ros-melodic-pinocchio 2>/dev/null | awk '/^Version/{print $2}')
[ -n "$v" ] && pass "pinocchio $v" || fail "pinocchio missing" "sudo apt install ros-melodic-pinocchio"
for l in libtoppra libfcl; do ldconfig -p | grep -q "$l\.so" && pass "$l in linker cache" || fail "$l not in linker cache" "sudo ldconfig"; done

echo "--- workspace (build.sh)"
for p in rci_holistic_controller sca_pkg rci_action_manager brl_interfaces summit_ur5_hardware_setup; do
  rospack find $p >/dev/null 2>&1 && pass "package $p" || fail "package $p not found" "build.sh, then source <ws>/devel/setup.bash"
done
for e in moveit_optimal_real rci_action_manager_real; do
  f=$(catkin_find --libexec rci_action_manager $e 2>/dev/null | head -1)
  if [ -z "$f" ]; then fail "executable $e missing" "build.sh (look for the first 'error:' in its output)"
  elif ldd "$f" | grep -q "not found"; then fail "$e: missing shared libraries: $(ldd "$f" | awk '/not found/{print $1}' | xargs)" "sudo ldconfig; else rerun install_deps.sh"
  else pass "executable $e (all libraries resolve)"; fi
done

echo "--- python 2 nodes"
python -c "import numpy, yaml, rospy, tf, actionlib" 2>/dev/null && pass "python imports numpy yaml rospy tf actionlib" \
  || fail "python imports" "sudo apt install python-numpy python-yaml (ROS python packages come with ros-melodic-*)"
for f in rci_action_manager/scripts/log_action_telemetry.py rci_action_manager/scripts/ur_wrench_to_wrist.py \
         hardware_setup/ft_gravity.py hardware_setup/prepare_field_ur5e_model.py hardware_setup/brl_lab/brl_hqp_servo.py; do
  full="$D/../../$f"
  if ! python -m py_compile "$full" 2>/tmp/hqp_check_py.err; then fail "$f does not compile on $(python --version 2>&1)" "$(tail -1 /tmp/hqp_check_py.err) -- use the melodic-devel branch"
  elif [ ! -x "$full" ]; then fail "$f not executable" "chmod +x $full"
  else pass "$f"; fi
done

echo "--- runtime packages used by the field launch files"
for p in controller_manager image_transport compressed_depth_image_transport tf; do
  rospack find $p >/dev/null 2>&1 && pass "$p" || fail "$p missing" "sudo apt install ros-melodic-${p//_/-}"
done
for l in ur5e_moveit_field.launch ur5e_actions_field.launch; do
  out=$(roslaunch --nodes summit_ur5_hardware_setup $l full_model:=false 2>&1) && pass "$l parses" \
    || fail "$l does not parse" "$(echo "$out" | tail -1)"
done

if [ "$MODE" = --live ]; then
  echo "--- live robot ($ROS_MASTER_URI, namespace $NS)"
  if ! timeout 5 rosparam list >/dev/null 2>&1; then
    fail "ROS master $ROS_MASTER_URI unreachable" "start the robot bringup; check ROS_MASTER_URI / ROS_IP and ping"
  else
    pass "ROS master reachable"
    hz() { timeout 6 rostopic hz -w 50 "$1" 2>/dev/null | awk '/average rate/{print int($3); exit}'; }
    r=$(hz $NS/joint_states); [ -n "$r" ] && [ "$r" -ge 100 ] && pass "$NS/joint_states ${r} Hz" \
      || fail "$NS/joint_states ${r:-no data} Hz (need >= 100)" "driver running? namespace right? (check.sh --live <ns>)"
    r=$(hz $NS/wrench); [ -n "$r" ] && pass "$NS/wrench ${r} Hz" || fail "$NS/wrench no data" "UR driver publishes the F/T wrench; check the bringup"
    f=$(timeout 3 rostopic echo -n1 $NS/wrench/header/frame_id 2>/dev/null | head -1 | tr -d '"')
    [ "$f" = ur5e_tool0_controller ] && pass "wrench frame $f" || warn "wrench frame '$f'" "launch files expect ur5e_tool0_controller"
    p=$(timeout 3 rostopic echo -n1 $NS/ur_hardware_interface/robot_program_running/data 2>/dev/null | head -1)
    [ "$p" = True ] && pass "External Control program running" || fail "robot_program_running='$p'" "start External Control on the teach pendant"
    s=$(timeout 3 rostopic echo -n1 $NS/speed_scaling_factor/data 2>/dev/null | head -1)
    [ -n "$s" ] && awk "BEGIN{exit !($s>0)}" && pass "speed_scaling_factor $s" || warn "speed_scaling_factor '${s:-no data}'" "speed slider 0 or the scaling controller not running"
    c=$(timeout 5 rosservice call $NS/controller_manager/list_controllers 2>/dev/null)
    echo "$c" | grep -A2 'name: "ur5e_scaled_pos_joint_traj_controller"' | grep -q 'state: "running"' \
      && pass "ur5e_scaled_pos_joint_traj_controller running" || fail "ur5e_scaled_pos_joint_traj_controller not running" \
      "rosservice call $NS/controller_manager/list_controllers  (the HQP sends its commands to this controller)"
    rosparam get /brl_robot/robot_description >/dev/null 2>&1 && pass "/brl_robot/robot_description present (full_model)" \
      || warn "/brl_robot/robot_description missing" "moveit launch default full_model:=true needs it; else full_model:=false"
  fi
fi
echo; echo "summary: $FAILS FAIL, $WARNS WARN"; [ $FAILS = 0 ]

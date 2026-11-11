#!/bin/bash
# Robot-free test of the HQP controller on this PC. Usage: selftest.sh [moveit|actions|all]  (default all)
# Runs the unchanged field launch files against fake_ur_driver.py on a PRIVATE ROS master
# (localhost:11399), so it is safe while the robot or the desktop master is running.
# Results: ~/.ros/hqp_selftest/<date>/  (console logs, result_*.json, telemetry CSV)
D=$(cd "$(dirname "$0")" && pwd)
MODES=${1:-all}; [ "$MODES" = all ] && MODES="moveit actions"
OUT=$HOME/.ros/hqp_selftest/$(date +%Y%m%d_%H%M%S); mkdir -p "$OUT"
export ROS_MASTER_URI=http://localhost:11399 ROS_HOSTNAME=localhost; unset ROS_IP
if ! rospack find rci_action_manager >/dev/null 2>&1; then
  echo "FAIL: workspace not sourced. Run: source ~/catkin_ws/devel/setup.bash"; exit 1; fi
cleanup() { [ -n "$LP" ] && kill -INT $LP 2>/dev/null; sleep 2; [ -n "$CP" ] && kill $CP 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT
roscore -p 11399 > "$OUT/roscore.log" 2>&1 & CP=$!
for i in $(seq 1 20); do rosparam list >/dev/null 2>&1 && break; sleep 0.5; done
rosparam list >/dev/null 2>&1 || { echo "FAIL: private roscore did not start (see $OUT/roscore.log)"; exit 1; }
# the field moveit launch reads the BRL robot model at load time; here: the arm URDF
rosparam set /brl_robot/robot_description -- "$(cat "$D/../field/ur5e_arm.urdf")"
status=0; PY=$(command -v python || command -v python3)
for m in $MODES; do
  echo "=== $m: $( [ $m = moveit ] && echo ur5e_moveit_field.launch || echo ur5e_actions_field.launch ) + fake UR driver"
  roslaunch "$D/selftest.launch" mode:=$m out_dir:="$OUT/$m" > "$OUT/$m.launch.log" 2>&1 & LP=$!
  $PY "$D/selftest.py" $m "$OUT/result_$m.json" "$D/selftest_reference.json" 2>"$OUT/$m.client.err" || status=1
  kill -INT $LP 2>/dev/null; wait $LP 2>/dev/null; LP=
  if ls "$OUT/$m/telemetry"/*/*.csv >/dev/null 2>&1 || ls "$OUT/$m/telemetry"/*.csv >/dev/null 2>&1; then
    echo "[PASS] telemetry CSV written      $(ls "$OUT/$m/telemetry"/*/*.csv "$OUT/$m/telemetry"/*.csv 2>/dev/null | head -1)"
  else echo "[FAIL] telemetry CSV written      (none under $OUT/$m/telemetry)"; status=1; fi
  if grep -qE "process has died|Traceback|\[FATAL\]" "$OUT/$m.launch.log"; then
    echo "[FAIL] launch log clean          see $OUT/$m.launch.log:"; grep -E "process has died|Traceback|\[FATAL\]|Error" "$OUT/$m.launch.log" | head -5; status=1
  else echo "[PASS] launch log clean"; fi
done
echo; [ $status = 0 ] && echo "SELFTEST PASS  (logs: $OUT)" || echo "SELFTEST FAIL  (logs: $OUT)"
exit $status

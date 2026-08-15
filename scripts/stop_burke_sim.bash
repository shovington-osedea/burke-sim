#!/usr/bin/env bash

# Stop the processes used by the Burk-e Gazebo simulation. This does not stop
# arbitrary ROS nodes, but it intentionally stops robot_state_publisher too so
# an orphaned launch cannot keep publishing stale simulation transforms.
set -u

patterns=(
  'ros2 launch burke_gazebo'
  'gz sim'
  'gzserver'
  'gzclient'
  'ign gazebo'
  'ros_gz_bridge/parameter_bridge'
  'ros_gz_sim/create'
  'robot_state_publisher'
)

pids=()
for pattern in "${patterns[@]}"; do
  while read -r pid; do
    [[ -n "$pid" && "$pid" != "$$" ]] && pids+=("$pid")
  done < <(pgrep -f "$pattern" || true)
done

if ((${#pids[@]} == 0)); then
  echo "No Burk-e Gazebo processes found."
  exit 0
fi

mapfile -t pids < <(printf '%s\n' "${pids[@]}" | sort -nu)
if ((${#pids[@]} == 0)); then
  echo "No Burk-e Gazebo processes found."
  exit 0
fi

echo "Stopping Burk-e Gazebo processes: ${pids[*]}"
kill -TERM "${pids[@]}" 2>/dev/null || true

for _ in {1..20}; do
  alive=()
  for pid in "${pids[@]}"; do
    kill -0 "$pid" 2>/dev/null && alive+=("$pid")
  done
  ((${#alive[@]} == 0)) && exit 0
  sleep 0.1
done

echo "Force-stopping remaining processes: ${alive[*]}"
kill -KILL "${alive[@]}" 2>/dev/null || true

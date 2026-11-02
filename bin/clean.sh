pkill -f gz-sim; pkill -f "gz sim"; pkill -f parameter_bridge
# gz-sim-main outlives ros2 launch; wait for it to exit, then force-kill stragglers
for _ in $(seq 1 10); do pgrep -f "gz-sim|gz sim|parameter_bridge" > /dev/null || break; sleep 1; done
pkill -9 -f "gz-sim|gz sim|parameter_bridge"
pgrep -af "gz-sim|gz sim|parameter_bridge"

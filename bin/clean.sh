# Stop everything a localization_sim launch can leave behind.
# Sourced by the test scripts, so no `exit` and no leftover variables.

_CLEAN_PATTERN='gz-sim|gz sim|parameter_bridge|install/devol_|octomap_server_node|robot_state_publisher|static_transform_publisher|ros2 topic echo|ros2 launch devol_'

# Ask nicely first so nodes can flush their logs
pkill -f "$_CLEAN_PATTERN"

# gz-sim-main and some nodes outlive ros2 launch; wait for them, then force-kill stragglers
for _ in $(seq 1 10); do
    pgrep -f "$_CLEAN_PATTERN" > /dev/null || break
    sleep 1
done
pkill -9 -f "$_CLEAN_PATTERN"
sleep 1

# Report anything that survived
if pgrep -af "$_CLEAN_PATTERN"; then
    echo "  [!] Warning: the processes above survived cleanup."
fi

unset _CLEAN_PATTERN
#!/bin/bash

# Source ROS and workspace environments
. /opt/ros/lyrical/setup.bash
. bin/clean.sh
. bin/set_env.sh

# Build the workspace
echo "Building the workspace..."
colcon build
. install/setup.bash

# Configuration for the PILOT test harness
NUM_RUNS=15  # 10 to 15 is a great sample size for a pilot
BASE_RESULTS_DIR="pilot_results"

# Pick the WORST-CASE configuration to find the maximum variance
NUM_PARTICLES=100
NOISE_SCALE=4.0
KIDNAP="true"
NOISE="true"

echo "Starting PILOT tests to determine N. $NUM_RUNS total runs scheduled."
echo "Configuration: $NUM_PARTICLES particles, noise $NOISE_SCALE, kidnapped=$KIDNAP"
echo "========================================================"

for SEED in $(seq 1 $NUM_RUNS); do
    echo "  -> Executing Pilot Run $SEED of $NUM_RUNS (Seed: $SEED)"

    # Save everything into a flat directory for the python script to easily parse
    RUN_RESULTS_DIR="${BASE_RESULTS_DIR}/run_${SEED}"
    mkdir -p "$RUN_RESULTS_DIR"

    timeout 300 ros2 launch devol_localization localization_sim.launch.py \
        pf_num_particles:="$NUM_PARTICLES" \
        kidnap:="$KIDNAP" \
        noise:="$NOISE" \
        noise_scale:="$NOISE_SCALE" \
        seed:="$SEED" \
        results_dir:="$RUN_RESULTS_DIR" \
        show_vis:="false" \
        log_metrics:="true" \
        monitor_goal:="true"

    EXIT_CODE=$?
    if [ $EXIT_CODE -eq 124 ]; then
        echo "  [!] Warning: Run $SEED timed out."
    elif [ $EXIT_CODE -ne 0 ]; then
        echo "  [!] Warning: Run $SEED exited with status $EXIT_CODE."
    else
        echo "  [X] Run $SEED complete."
    fi

    . bin/clean.sh
done

echo "========================================================"
echo "Pilot testing complete. Results are in '${BASE_RESULTS_DIR}'."
echo "Run the python analysis script on this folder to calculate N."
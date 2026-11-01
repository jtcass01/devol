#!/bin/bash

# Source ROS and workspace environments
. /opt/ros/lyrical/setup.bash
. bin/clean.sh
. bin/set_env.sh

# Build the workspace
echo "Building the workspace..."
colcon build
. install/setup.bash

# Configuration for the test harness
NUM_RUNS=10
BASE_RESULTS_DIR="test_results"

# Array of particle counts to test
PARTICLE_COUNTS=(100 500 2000 5000)

TOTAL_RUNS=$((${#PARTICLE_COUNTS[@]} * 2 * NUM_RUNS))
echo "Starting automated tests: $TOTAL_RUNS total runs scheduled."

# Loop over particle counts
for NUM_PARTICLES in "${PARTICLE_COUNTS[@]}"; do
    echo "========================================================"
    echo "Testing with $NUM_PARTICLES particles"
    echo "========================================================"

    # Loop over the kidnapping conditions
    for KIDNAP in "false" "true"; do
        
        if [ "$KIDNAP" == "true" ]; then
            CONDITION_NAME="kidnapped"
        else
            CONDITION_NAME="baseline"
        fi

        echo "--------------------------------------------------------"
        echo "Running condition: $CONDITION_NAME ($NUM_PARTICLES particles)"
        echo "--------------------------------------------------------"

        for SEED in $(seq 1 $NUM_RUNS); do
            echo "  -> Executing Run $SEED of $NUM_RUNS (Seed: $SEED, Particles: $NUM_PARTICLES, Kidnap: $KIDNAP)"

            RUN_RESULTS_DIR="${BASE_RESULTS_DIR}/particles_${NUM_PARTICLES}/${CONDITION_NAME}/run_${SEED}"
            mkdir -p "$RUN_RESULTS_DIR"

            timeout 300 ros2 launch devol_localization localization_sim.launch.py \
                pf_num_particles:="$NUM_PARTICLES" \
                kidnap:="$KIDNAP" \
                seed:="$SEED" \
                results_dir:="$RUN_RESULTS_DIR" \
                show_vis:="false" \
                show_vis:="false" \
                log_metrics:="true" \
                monitor_goal:="true" \
                goal3_x:="10.0" \
                goal3_y:="-5.0"
                
            # Check if the run crashed, timed out, or completed successfully
            EXIT_CODE=$?
            if [ $EXIT_CODE -eq 124 ]; then
                echo "  [!] Warning: Run $SEED timed out after 5 minutes."
            elif [ $EXIT_CODE -ne 0 ]; then
                echo "  [!] Warning: Run $SEED exited with status $EXIT_CODE."
            else
                echo "  [X] Run $SEED complete."
            fi
            
            echo "  Results saved to $RUN_RESULTS_DIR"
            echo ""
        done
    done
done

echo "========================================================"
echo "All $TOTAL_RUNS runs completed."
echo "Results are organized in the '${BASE_RESULTS_DIR}' directory."
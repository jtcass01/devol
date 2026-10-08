# usage: run_tc.sh <test_case>   (inside devol-gzt; seed 0, viz off, CPU sampled, particles logged)
source /opt/ros/lyrical/setup.bash; source /deps/install/setup.bash; source /ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/gzmodels:/deps/install/share:$GZ_SIM_RESOURCE_PATH; export LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=offscreen MPLBACKEND=Agg
n=$1; o=/out/tc$n; mkdir -p $o
python3 /tools/cpusample.py $o/cpu.json & CS=$!
python3 /tools/particle_logger.py $o/particles.npz & PL=$!
timeout 2400 ros2 launch devol_localization localization_test_cases.launch.py test_case:=$n seed:=0 viz:=false record_video:=false check_running_sim:=false output_dir:=$o > $o/launch.log 2>&1; echo "exit $?" >> $o/launch.log
kill -INT $PL; touch $o/cpu.json.stop; wait $PL $CS

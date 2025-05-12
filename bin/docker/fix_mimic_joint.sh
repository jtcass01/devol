if [ ! -f .env ]; then
    . ./bin/docker/generate_env.sh
fi
. .env

docker exec -u root $CONTAINER_NAME bash -c '
    file_path="/opt/ros/jazzy/share/robotiq_description/urdf/robotiq_gripper.ros2_control.xacro";
    sed -i "89d" $file_path;
    sed -i "80d" $file_path;
    sed -i "71d" $file_path;
    sed -i "62d" $file_path;
    sed -i "53d" $file_path;
'

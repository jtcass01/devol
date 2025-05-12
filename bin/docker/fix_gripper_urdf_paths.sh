if [ ! -f .env ]; then
    . ./bin/docker/generate_env.sh
fi
. .env

docker exec -u root $CONTAINER_NAME bash -c '
base_path="/opt/ros/jazzy/share/robotiq_description/urdf"
for file in $(ls /opt/ros/jazzy/share/robotiq_description/urdf/); do
    full_path="$base_path/$file";
    sed -i "s|package://robotiq_description|file:///opt/ros/jazzy/share/robotiq_description|g" "$full_path"
done
'

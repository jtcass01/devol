# Generate the .env file if it doesn't exist, then source it
if [ ! -f .env ]; then
    . ./bin/docker/generate_env.sh
fi
. .env

COMPOSE_BAKE=true docker compose build

. bin/docker/start.sh

. bin/docker/fix_gripper_urdf_paths.sh

. bin/docker/fix_mimic_joint.sh
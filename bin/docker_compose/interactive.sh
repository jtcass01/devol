# Generate the .env file if it doesn't exist, then source it
if [ ! -f .env ]; then
    . ./bin/docker/generate_env.sh
fi
. .env

xhost +
docker exec -it $CONTAINER_NAME bash # Login to an interactive window

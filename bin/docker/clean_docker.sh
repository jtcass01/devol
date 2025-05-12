# Generate the .env file if it doesn't exist, then source it
if [ ! -f .env ]; then
    . ./bin/docker/generate_env.sh
fi
. .env

# Grab the image name from the container before we shut it down
IMAGE_NAME=`docker container ls --filter "name=$CONTAINER_NAME" --format "{{.Image}}"`
# The .env file will bring down the correct container and network
docker compose down --volumes
# Remove the image associated with the container
docker image rm ${IMAGE_NAME}
# Clean up dangling stuff (no -a or all your docker images go away :\ )
docker system prune --volumes --force

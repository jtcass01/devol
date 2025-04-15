# Generate the .env file if it doesn't exist, then source it
if [ ! -f .env ]; then
    . ./bin/docker/generate_env.sh
fi
. .env
# Simply stop the container (doesn't clean anything up)
docker compose down --volumes
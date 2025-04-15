# Generate the .env file if it doesn't exist, then source it
if [ ! -f .env ]; then
    . ./bin/docker/generate_env.sh
fi
. .env

COMPOSE_BAKE=true docker compose build

# Generate the .env file if it doesn't exist, then source it
if [ ! -f .env ]; then
    . ./bin/docker/generate_env.sh
fi
. .env
# The .env file ensures the COMPOSE_PROJECT_NAME is set before this call
DOCKER_DEFAULT_RUNTIME=nvidia docker compose up -d
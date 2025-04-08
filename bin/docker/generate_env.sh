# If the container and network names aren't specified in the docker-compose.yml,
# they will be generated from the service name. If COMPOSE_PROJECT_NAME is set,
# it will be prepended to the service name. The default container naming scheme
# is then ${COMPOSE_PROJECT_NAME}-${service_name}-1
cat << EOF > .env
DEVOL_WS=`pwd`
USER_UID=`id -u`
USER_GID=`id -g`
COMPOSE_PROJECT_NAME=${USER}
CONTAINER_NAME=${USER}-devol-dev
EOF
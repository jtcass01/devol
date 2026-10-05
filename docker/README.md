# devol Docker image

`Dockerfile` builds Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Jetty (through the `ros_gz` vendor packages)
with the workspace built into `/opt/devol_ws`. `compose.yaml` runs it on Docker Desktop for Windows
(WSL 2 backend), with the GUIs shown through WSLg and rendering on the GPU via Mesa's d3d12 driver.

- `sim`: the image as built. Test case results land in `results/docker/` in the repository.
- `dev`: the same, with the repository's `src/` mounted and build outputs kept in Docker volumes.
- `DEVOL_TAG` overrides the image tag (default `lyrical`), `ROS_DOMAIN_ID` (default 42) and
  `GZ_PARTITION` (default `devol_docker`) keep it apart from a native simulation.

Usage, from the repository root, is in the [main README](../README.md).

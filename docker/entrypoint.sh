#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi

# WSLg exposes its X11 socket under /mnt/wslg; link it where X clients look for it.
if [ -d /mnt/wslg/.X11-unix ] && [ ! -e /tmp/.X11-unix/X0 ]; then
    mkdir -p /tmp/.X11-unix
    ln -sf /mnt/wslg/.X11-unix/X0 /tmp/.X11-unix/X0
fi

# The GPU user-mode driver (libd3d12, libdxcore) comes from the host through /usr/lib/wsl.
if [ -d /usr/lib/wsl/lib ]; then
    export LD_LIBRARY_PATH="/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

exec "$@"

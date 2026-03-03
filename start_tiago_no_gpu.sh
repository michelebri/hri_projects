#!/usr/bin/env bash
set -euo pipefail

# Start TIAGo in a CPU-only container. No NVIDIA runtime flags are used.
# The software rendering env vars reduce the chance of accidental host GPU use.

IMAGE_NAME="${IMAGE_NAME:-spqr:booster}"
CONTAINER_NAME="${CONTAINER_NAME:-tiago_sim}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null
fi

docker run \
  --env DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --env LIBGL_ALWAYS_SOFTWARE=1 \
  --env MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "${SCRIPT_DIR}:/root/exchange" \
  --volume /var/run/docker.sock:/var/run/docker.sock \
  --env ROBOT_STACK=tiago \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --net host \
  --privileged \
  --rm \
  -it \
  --name "${CONTAINER_NAME}" \
  --workdir /root/exchange \
  "${IMAGE_NAME}" \
  /root/exchange/docker/entrypoint_tiago.sh

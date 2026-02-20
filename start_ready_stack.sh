#!/bin/bash
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-$(pwd)}"
SCENE_PATH="${CIRCUS_SCENE:-resources/scenes/solo.yaml}"

echo "[ready] project root: ${PROJECT_ROOT}"

cd "${PROJECT_ROOT}/simbridge"
if [ ! -x "${PROJECT_ROOT}/simbridge/.pixi/envs/default/bin/simbridge_node" ]; then
  echo "[ready] simbridge not built yet, running pixi install + build..."
  pixi install
  pixi run build
else
  echo "[ready] simbridge already built, skipping build."
fi

cd "${PROJECT_ROOT}/circus"
if [ ! -d "${PROJECT_ROOT}/circus/.pixi" ]; then
  echo "[ready] circus env not found, running pixi install..."
  pixi install
else
  echo "[ready] circus env already present, skipping install."
fi

echo "[ready] starting circus scene: ${SCENE_PATH}"
exec pixi run circus "${SCENE_PATH}"

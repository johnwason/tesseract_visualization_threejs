#!/bin/bash
# Build tesseract, its source dependencies and tesseract_visualization_threejs with colcon, in the container, then
# run the tesseract_visualization_threejs tests. All output is under build/. usage: tools/build.sh [--no-test]
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
exec "$ROOT/tools/run.sh" bash -euo pipefail -c '
  mkdir -p build/ccache
  export CMAKE_BUILD_PARALLEL_LEVEL='"${JOBS:-3}"'
  colcon --log-base build/log build \
    --build-base build/colcon --install-base build/install --merge-install \
    --base-paths src/ros_industrial_cmake_boilerplate src/boost_plugin_loader src/opw_kinematics src/tesseract tesseract_visualization_threejs \
    --packages-up-to tesseract_visualization_threejs --parallel-workers 1 --event-handlers console_direct- console_cohesion+ \
    --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
                 -DTESSERACT_VISUALIZATION_THREEJS_ENABLE_TESTING=ON
  if [ "'"${1:-}"'" != "--no-test" ]; then
    export LD_LIBRARY_PATH=/ws/build/install/lib
    ctest --test-dir build/colcon/tesseract_visualization_threejs --output-on-failure
  fi
'

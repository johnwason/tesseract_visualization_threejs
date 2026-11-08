#!/bin/bash
# Build the viewer against the normal tesseract install with sanitizers and run the C++ tests.
# Eigen picks a different allocator when __SANITIZE_ADDRESS__ is defined, which would not match the
# uninstrumented tesseract libraries, so EIGEN_MALLOC_ALREADY_ALIGNED pins it to their choice.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
# ThreadSanitizer needs address randomization off, which the default seccomp profile does not allow
export PODMAN_ARGS="--security-opt seccomp=unconfined"
exec "$ROOT/tools/run.sh" bash -uo pipefail -c '
status=0
run() { # name flags tests...
  name=$1; flags=$2; shift 2
  cmake -S tesseract_viewer -B build/$name -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_PREFIX_PATH=/ws/build/install \
    -DTESSERACT_VIEWER_ENABLE_TESTING=ON -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_FLAGS="$flags -fno-omit-frame-pointer -DEIGEN_MALLOC_ALREADY_ALIGNED=1" \
    -DCMAKE_EXE_LINKER_FLAGS="$flags" -DCMAKE_SHARED_LINKER_FLAGS="$flags" > /dev/null || exit 1
  CMAKE_BUILD_PARALLEL_LEVEL='"${JOBS:-4}"' cmake --build build/$name 2>&1 | tail -1
  for t in "$@"; do
    echo "== $name $t"
    ( cd build/$name
      LD_LIBRARY_PATH=/ws/build/install/lib:/ws/build/$name \
      TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=/ws/build/$name TESSERACT_VISUALIZATION_PLUGINS=tesseract_viewer_visualization_plugin \
      setarch -R ./$t > /ws/build/logs/$name.$t.log 2>&1 ); code=$?
    echo "exit=$code"; [ $code -eq 0 ] || status=1
    grep -E "PASSED|FAILED|ERROR: AddressSanitizer|WARNING: ThreadSanitizer|runtime error|^SUMMARY" /ws/build/logs/$name.$t.log | cut -c1-200 | sort | uniq -c | head -12
  done
}
export ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 TSAN_OPTIONS=halt_on_error=0:suppressions=/ws/tools/tsan.supp
run asan "-fsanitize=address,undefined" tesseract_viewer_unit tesseract_viewer_server_unit
run tsan "-fsanitize=thread" tesseract_viewer_server_unit
exit $status
'

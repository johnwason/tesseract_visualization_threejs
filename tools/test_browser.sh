#!/bin/bash
# Run the Playwright tests of the client against the C++ server, in the container.
# Screenshots are written to build/browser. Pass extra Playwright arguments, such as -g <name>.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
exec "$ROOT/tools/run.sh" bash -euo pipefail -c '
  export LD_LIBRARY_PATH=/ws/build/install/lib
  mkdir -p build/browser
  cd /opt/playwright
  npx playwright test --config /ws/tesseract_viewer/test/browser/playwright.config.js "$@"
' test_browser "$@"

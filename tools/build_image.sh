#!/bin/bash
# Build the persistent development image. Only rerun when tools/Containerfile changes.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
exec podman build -t tesseract-viewer-dev:noble --volume "$ROOT/src:/deps:ro" -f "$ROOT/tools/Containerfile" "$ROOT/tools"

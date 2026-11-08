# Working in the container

`src/` holds the upstream checkouts and `build/` all build output; both are git-ignored.

| Checkout | Version | Used for |
| --- | --- | --- |
| `src/tesseract` | `2b79e91c` | built |
| `src/ros_industrial_cmake_boilerplate` | `0.7.4` | built |
| `src/boost_plugin_loader` | `0.4.5` | built |
| `src/opw_kinematics` | `0.5.5` | built |
| `src/tesseract_python` | `cca2472` | source of the port, not built |
| `src/tesseract_planning` | `a6483823` | reference only, not built |

```bash
tools/build_image.sh    # once: bakes every apt and rosdep package into tesseract-visualization-threejs-dev:noble
tools/build.sh          # colcon build and ctest; JOBS and MEM limit it
tools/test_browser.sh   # Playwright: the client in Chromium; screenshots in build/browser
tools/test_sanitizers.sh # the C++ tests under ASan, UBSan and TSan
tools/run.sh bash       # a shell in the image

# Serve a robot on http://localhost:8000
tools/run.sh bash -c 'LD_LIBRARY_PATH=/ws/build/install/lib /ws/build/install/bin/tesseract_visualization_threejs_demo \
  /ws/src/tesseract/support/urdf/abb_irb2400.urdf /ws/src/tesseract/support/urdf/abb_irb2400.srdf'
```

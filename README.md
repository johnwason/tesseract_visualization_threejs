# tesseract_visualization_threejs

A web viewer for [Tesseract](https://github.com/tesseract-robotics/tesseract), in C++. It serves a three.js page that shows a Tesseract environment, animates trajectories and draws markers, in any modern browser and with no internet access.

It's a C++ port of `tesseract_viewer_python` from [tesseract_python](https://github.com/tesseract-robotics/tesseract_python), and it can be used as a Tesseract visualization plugin or as a library.

CI builds and tests it on Ubuntu 24.04 (x64 and arm64), macOS (x64 and arm64) and Windows x64.

> **Temporary dependency:** the plugin needs `VisualizationFactory`, which is proposed in [tesseract#1399](https://github.com/tesseract-robotics/tesseract/pull/1399). Until that merges, `dependencies.repos` pins a commit on `johnwason/tesseract`.

## Build

The build uses [colcon](https://colcon.readthedocs.io), as Tesseract does. On Ubuntu, with `vcstool` and `rosdep` installed:

```bash
mkdir -p ws/src && cd ws/src
git clone https://github.com/johnwason/tesseract_visualization_threejs.git
cd ..
vcs import src < src/tesseract_visualization_threejs/dependencies.repos
rosdep install --from-paths src -iry --skip-keys cereal
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Besides Tesseract, it needs jsoncpp, OpenSSL and Boost.Beast. Set `-DTESSERACT_VISUALIZATION_THREEJS_BUILD_SERVER=OFF` to build only the conversions and viewer state, without Boost.Beast. For a container with every dependency installed, see [tools/README.md](tools/README.md).

## Quick start

```cpp
#include <tesseract/visualization/threejs/threejs_visualization.h>

tesseract::visualization::threejs::ThreejsVisualization viewer;
viewer.plotEnvironment(env);    // starts the server and logs its URL, usually http://127.0.0.1:8000
viewer.plotTrajectory(trajectory, *env.getStateSolver());
viewer.waitForInput();
```

Each `ThreejsVisualization` object has its own server and scene.

## Use as a plugin

Applications that already use `VisualizationLoader`, such as the `tesseract_planning` examples, can use the viewer without code changes:

```bash
export TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=<install prefix>/lib
export TESSERACT_VISUALIZATION_PLUGINS=tesseract_visualization_threejs_plugin
```

To ask for it by name and configure it:

```cpp
tesseract::visualization::VisualizationLoader loader;
auto plotter = loader.get("TesseractThreejsVisualizationPlugin", YAML::Load("port: 8080"));
```

Each loader owns one instance, so a second loader gives an independent server and scene.

`get()` with no name asks for the Ignition plugin's symbol, which this plugin also exports. If the real Gazebo plugin is in the plugin list too, library search order decides which one loads.

## Configuration

| Setting | YAML | Environment variable | Default |
| --- | --- | --- | --- |
| Address to bind | `host` | `TESSERACT_VISUALIZATION_THREEJS_HOST` | `127.0.0.1` |
| Port, 0 for any free port | `port` | `TESSERACT_VISUALIZATION_THREEJS_PORT` | First free port from 8000 |

YAML takes precedence over the environment.

## Use as a library

The code is in layers, so an application can use only what it needs. Everything is in the `tesseract::visualization::threejs` namespace.

| Layer | Header | Library | Provides |
| --- | --- | --- | --- |
| Conversions | `conversions.h` | `tesseract_visualization_threejs` | Environment to glTF and GLB, trajectories and markers to JSON |
| Viewer state | `viewer_state.h` | `tesseract_visualization_threejs` | Current scene, trajectory and markers, their ETags, and websocket messages, with no networking |
| Server | `viewer_server.h` | `tesseract_visualization_threejs_server` | HTTP and websocket server for a `ViewerState` |
| Visualization | `threejs_visualization.h` | `tesseract_visualization_threejs_server` | The `Visualization` interface, on top of the layers above |

Use `ViewerState` directly to serve the viewer over another transport.

## Examples

Run these after sourcing the workspace, with `TESSERACT_RESOURCE_PATH` set so that `package://tesseract` URLs resolve (the `share` directory of the install works). Then open the URL they log.

| Program | Shows |
| --- | --- |
| `tesseract_visualization_threejs_shapes_viewer` | Primitive shapes from an inline URDF |
| `tesseract_visualization_threejs_material_mesh_viewer` | Textured glTF and Collada meshes |
| `tesseract_visualization_threejs_abb_irb2400_viewer` | A robot, an animated trajectory, and the tool path with axes |
| `tesseract_visualization_threejs_viewer_demo <urdf> <srdf> [host] [port]` | Any robot, with a sweep of its joints and two markers |

Add `?lighting=environment` to the viewer URL, for example `http://127.0.0.1:8000/?lighting=environment`, to light the scene with an environment map as well as the hemisphere light. Metallic and rough PBR materials, and normal maps, then look as intended. The default is the hemisphere light alone, which gives CAD meshes more contrast.

Two pages in [examples/javascript](tesseract_visualization_threejs/examples/javascript) embed the viewer in an iframe and control it with `postMessage`: `iframe_joint_positions.html` and `iframe_trajectory.html`. Start the ABB example, then open either page.

The Python ABB example plans with OMPL and TrajOpt. The C++ version interpolates joint positions instead, so that this project doesn't depend on `tesseract_planning`. The Python HTTPS WebXR example has no C++ version, because the server doesn't support TLS yet.

## Tests

```bash
tools/build.sh             # build, and run the C++ tests
tools/test_browser.sh      # run the client in Chromium with Playwright
tools/test_sanitizers.sh   # run the C++ tests under ASan, UBSan and TSan
```

The browser tests compare the scene three.js builds with Tesseract's forward kinematics, run every example, and block all external requests.

## Limitations

- The viewer is display only: clients can't send commands back to the application.
- There's no authentication or TLS, so it binds to localhost by default. Only bind other addresses on a network you trust.
- WebXR needs HTTPS, so VR headsets can't use it yet. Controller models would also load from a CDN.
- Geometry markers are always white, because Tesseract's `GeometryMarker` has no color. Capsules are sent, but the client doesn't draw them yet.

[PORTING_NOTES.md](PORTING_NOTES.md) lists the marker mapping and every difference from the Python viewer.

## License

Apache License 2.0, as declared in `package.xml`. The bundled three.js (MIT) and robust-websocket (ISC) keep their own licenses, in [resources/static/vendor](tesseract_visualization_threejs/resources/static/vendor).

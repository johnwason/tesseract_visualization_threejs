# tesseract_viewer_cpp handoff

Goal: move the tesseract viewer's scene conversion and command logic from Python into a C++ library, so that tesseract_robotraconteur and other C++ projects can drive the viewer without Python. The JavaScript client stays as it is.

Status: nothing is written yet. This directory is an empty git repository (`main`, identity John Wason <wason@wasontech.com>) holding only this file.

## Decisions so far

- **Keep the JavaScript client unchanged.** That is `index.html` and `app.js` (31 KB, three.js) from tesseract_viewer_python. The C++ output must match the existing wire format exactly: glTF and GLB, the trajectory JSON, the markers JSON and the websocket commands.
- **Move to C++:** the environment-to-glTF conversion, the trajectory and joint-state conversion, the markers, and the viewer state and commands.
- **Implement the server as a `tesseract::visualization::Visualization` plugin.** tesseract_planning's C++ examples already call that interface, so every C++ project that loads a visualization plugin gets the viewer.

## Proposed layering

1. **Conversion: pure functions, no networking.**
   - From an Environment: glTF JSON plus binary buffer, and GLB.
   - A `SceneState` to the joint-positions trajectory JSON.
   - A `JointTrajectory` to the trajectory JSON. The glTF animation goes here too.
   - `tesseract::visualization` markers to the markers JSON.
2. **Viewer state and commands, independent of transport.** This holds the current scene GLB, trajectory and markers with their content hashes, used as ETags, and generates the websocket messages. tesseract_robotraconteur uses this layer directly and serves it over Robot Raconteur, with no HTTP.
3. **Optional HTTP and websocket server**, implemented with Boost.Beast. It serves the static files and the endpoints below and implements the `Visualization` plugin on top of layers 1 and 2. It binds to localhost by default.
4. **Python bindings** to layers 1 and 2, in nanobind (tesseract_nanobind) and SWIG (tesseract_python). The existing Python server can then become a thin transport over them, or be replaced by layer 3.

## The Python code being ported

The source is `tesseract_viewer_python/tesseract_robotics_viewer/` in https://github.com/tesseract-robotics/tesseract_python. A local copy at commit `cca2472` (2026-06-01) is in `/home/wasonj/tesseract/.wheel_work/up/tp/tesseract_viewer_python`.

| File | Lines | What it does |
| --- | --- | --- |
| `tesseract_env_to_gltf.py` | 608 | `tesseract_env_to_gltf`, `tesseract_env_to_glb`, `tesseract_env_to_gltf_dict_and_buf`: walks the links and joints recursively and writes nodes, meshes, materials, accessors and buffer views, plus `_append_trajectory_animation` |
| `util.py` | 65 | `tesseract_trajectory_to_list`, `trajectory_list_to_json`, `joint_positions_to_trajectory_json`, `trajectory_list_to_frames` |
| `tesseract_viewer_aio.py` | 771 | aiohttp server, websocket connections, marker builders (axes, arrow, box, sphere, cylinder, capsule, lines), clearing markers by tag or name, `plot_trajectory` |
| `tesseract_viewer.py` | 556 | a synchronous wrapper around the aio viewer |
| `resources/geometries.json` | 169 KB | pre-tessellated primitive meshes (cube, sphere, cylinder and so on). Compile it into the library or install it as data |
| `resources/static/{index.html,app.js}` | — | the client. Unchanged |

### Wire format the client expects (keep it identical)

**HTTP routes**, all supporting HEAD with an ETag (`hash_bytes`):
- `GET /` and `/index.html`
- `/app.js`
- `/tesseract_scene.gltf`
- `/tesseract_scene.glb`
- `/tesseract_trajectory.json`
- `/tesseract_markers.json`
- `/websocket`

**Websocket messages** are JSON of the form `{"command": ..., "params": ...}`. The server sends:
- `refresh_scene`, with no params, after the environment changes
- `joint_trajectory`, with the trajectory JSON
- `markers`, with the markers JSON

`app.js` also handles `refresh_trajectory`, `refresh_markers` and `joint_positions`.

**Trajectory JSON:**
```json
{"use_time": true, "loop_time": 20, "joint_names": [...], "trajectory": [[q0, ..., qn, t], ...]}
```
A static joint state is sent as a two-point trajectory: the positions with `t = 0`, then again with `t = 1e6`, with `loop_time` 10000.

**Marker dict:**
```json
{"marker_type": ..., "parent_link_name": "world", "position": [...], "quaternion": [...], "name": ..., "color": [r, g, b, a], "tags": [...]}
```
Each marker type adds its own fields, for example size, radius, length, vertices or linewidth. The `_fix_marker_*` helpers normalize vectors, colors and quaternions, so read them for the exact conventions. The whole document is `{"markers": [...]}`.

### Porting notes

- **cv2 can be dropped.** It's used once, at `tesseract_env_to_gltf.py:250`, to decode texture images. glTF embeds PNG and JPEG bytes directly with a `mimeType`, so no image library is needed unless other texture formats have to be supported.
- **Test against the Python output.** Generate glTF/GLB, trajectory and markers JSON with the Python code for the tesseract support models (for example `abb_irb2400`, and the material and textured mesh examples in `tesseract_viewer_python/examples/`). Compare the C++ output against those files structurally, not byte for byte, since floats and key order will differ.
- **JSON library:** prefer one the tesseract stack or vcpkg already provides, rather than adding another dependency.

## The tesseract interface to implement

This is from the pinned tesseract checkout in `/home/wasonj/tesseract/.wheel_work/src/tesseract/visualization`, commit `2b79e91c`.

`include/tesseract/visualization/visualization.h` declares `tesseract::visualization::Visualization`:
- `isConnected`, `waitForConnection(long seconds)`
- `plotEnvironment(const Environment&, ns)`, `plotEnvironmentState(const SceneState&, ns)`
- `plotTrajectory(const JointTrajectory&, const StateSolver&, ns)`
- `plotMarker(const Marker&, ns)`, `plotMarkers(const std::vector<std::shared_ptr<Marker>>&, ns)`
- `clear(ns)`, `waitForInput(message)`

**Markers.** `markers/marker.h` defines the `MarkerType` enum: GEOMETRY, AXIS, ARROW, LINE_STRIP, LINE_LIST, POINTS, TEXT, TRIANGLE_*, TOOLPATH, CONTACT_RESULTS and USER_DEFINED. The headers present are arrow, axis, contact_results, geometry and toolpath. Map each onto the client's marker types, and record the ones the client can't show.

**Prior art.** tesseract's own Ignition visualization, in `include/tesseract/visualization/ignition/{conversions.h,entity_manager.h}` and `src/ignition/`, shows how tesseract converts a scene for a viewer and keeps track of its entities.

**Plugin loading.** `VisualizationLoader` is a boost_plugin_loader `PluginLoader` that reads `TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES` and `TESSERACT_VISUALIZATION_PLUGINS`. With an empty name, `get()` loads the Ignition symbol (`TESSERACT_IGNITION_SYMBOL_NAME`).

## Packaging constraints (from the tesseract_nanobind wheel work)

- **How the PyPI wheels are built.** Branch `dev/nanobind_wheels`, PR pending against tesseract-robotics/tesseract_nanobind from the fork johnwason/tesseract_nanobind. The wheels use vcpkg only, with no conda or pixi. tesseract is built from source with colcon at the pins in `tesseract_nanobind/dependencies.rosinstall`, and the libraries are bundled with auditwheel, delvewheel or delocate.
- **This library joins that build.** It becomes one more colcon package in `dependencies.rosinstall`. If it uses Boost.Beast, `boost-beast` is added to the vcpkg package list (`VCPKG_PKGS` in `wheels.yml`).
- **The plugin needs an anchor.** The repair tools only bundle libraries that something links against. tesseract_nanobind links the 9 factory libraries into its modules and registers them with `addSymbolLibraryToSearchLibrariesEnv`, as tesseract_python's SWIG `%init` does. Do the same for this plugin, using `tesseract_nb_find_plugin_target` in tesseract_nanobind's `CMakeLists.txt`.
- **Static files.** The client files and `geometries.json` are either compiled in, which is simplest for wheels, or installed under `share/` and located by the Python package. Importing the bindings must not modify the environment, and nothing may add resource paths automatically.
- **Platforms:** Linux x64 and arm64 (`manylinux_2_39`), Windows x64, and macOS x64 and arm64, built on GitHub Actions runners. Watch for `<stdexcept>`-style missing includes. The vcpkg Boost is less forgiving than conda's.

## Context

- **Consumers:** tesseract_robotraconteur and other C++ projects, which use layers 1 and 2. The tesseract_nanobind and tesseract_python bindings use layers 1, 2 and optionally 3.
- **tesseract_nanobind's current viewer is unrelated.** It's a separate viser-based Python viewer (`src/tesseract_robotics/viewer`) that depends on viser, trimesh and pillow, and this C++ library doesn't replace it directly. Making it an optional `[viewer]` extra was suggested as a separate PR.
- **Open question:** does the viewer need to send input back to the application (GUI controls calling back into C++ or Python), or is it display only? Display only is assumed so far, which keeps layer 3 simple.

## Suggested first steps

1. Choose the repository layout. Proposal: one `tesseract_viewer` repository containing the C++ library, the static assets copied from tesseract_viewer_python, and the plugin, with ros_industrial_cmake_boilerplate like the other tesseract packages.
2. Write the layer 1 and 2 public header: conversion function signatures and a `ViewerState` class. Take the signatures from the Python functions above, and review them before porting.
3. Generate the reference outputs from the Python viewer for the support models, to use as test data.
4. Port `tesseract_env_to_gltf.py` and `util.py`, then the markers, then the state and commands, then the Beast server and plugin.
5. Add Python bindings, then add the package to tesseract_nanobind's `dependencies.rosinstall` with a plugin anchor.

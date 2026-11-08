# tesseract_viewer: port notes

The C++ replacement for the scene conversion, viewer state and server in `tesseract_viewer_python`. The JavaScript client in `tesseract_viewer/resources/static` comes from the Python viewer (`tesseract_python` commit `cca2472`), with the two fixes listed below.

## Layout

| Path | Contents |
| --- | --- |
| `tesseract_viewer/include/tesseract/viewer/conversions.h` | Layer 1: environment to glTF/GLB, trajectories, marker conversion |
| `tesseract_viewer/include/tesseract/viewer/viewer_state.h` | Layer 2: documents, ETags and websocket messages, no transport |
| `tesseract_viewer/include/tesseract/viewer/viewer_server.h` | Layer 3: Boost.Beast HTTP and websocket server |
| `tesseract_viewer/src/viewer_visualization_plugin.cpp` | The `Visualization` plugin |
| `tesseract_viewer/test/` | gtest suites, and the Playwright suite in `test/browser` |
| `tools/` | Container image and build scripts; see `tools/README.md` |

Dependencies: tesseract, jsoncpp, OpenSSL (SHA-256 and base64), and Boost.Beast for layer 3. A packaged build needs `boost-beast` and `openssl` added to `VCPKG_PKGS`.

## Using the plugin

```bash
export TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=<prefix>/lib
export TESSERACT_VISUALIZATION_PLUGINS=tesseract_viewer_visualization_plugin
# Optional. Defaults are 127.0.0.1 and 8000
export TESSERACT_VIEWER_HOST=127.0.0.1 TESSERACT_VIEWER_PORT=8000
```

The symbol is `TesseractViewerVisualizationPlugin`. The class is also exported under the Ignition symbol, so existing `VisualizationLoader::get()` callers get the viewer without code changes. For wheels, link the plugin through `ViewerVisualizationAnchor` in `viewer_visualization_anchor.h`.

## Changes to the client

- `addConesMarker` is renamed `addConeMarker`, the name `setMarkers` calls. Before this, a cone threw and dropped every marker after it.
- The scene, trajectory and marker timers in `createScene` and `updateScene` are stored on `this`. They were stored on `self`, the window, so they could not be cleared.

Triangle mesh normals are still written to the buffer but not referenced, as in the Python viewer, which has that reference commented out.

## Deliberate differences from the Python viewer

| Python | C++ |
| --- | --- |
| HEAD on `/tesseract_trajectory.json` returned 500 once a trajectory was set (missing `return r`) | Returns 200 with the ETag |
| `origin_offset` was accepted and ignored | Parameter removed |
| An unsupported visual geometry raised `UnboundLocalError` | Logged and skipped; visual numbering is unaffected |
| Non-triangle faces produced corrupt indices | Fan-triangulated |
| FK passed the time column to `calcFwdKin`, and ignored joint names | `trajectoryToFrames` matches columns by name and drops the time |
| cv2 decoded each texture and discarded the result | No image library |
| `plot_trajectory` sent the markers twice | Sent once |
| Key order and float formatting from Python's `json` | jsoncpp's; equivalent once parsed |
| Primitive meshes used the invalid attribute `NORMALS`, so three.js ignored their normals | `NORMAL`. The normals in `geometries.json` are per-face, so primitives still look faceted |
| JPEG textures were labelled `image/jpg`, and the type came from the URL, so embedded PNGs were labelled as JPEG | `image/jpeg` or `image/png`, from the image data |
| Accessors could be unaligned after image data in the buffer | Padded to a four-byte boundary |

TLS is not implemented, so the WebXR example's HTTPS mode has no C++ equivalent yet.

## Marker mapping

| tesseract marker | Client marker | Notes |
| --- | --- | --- |
| `AxisMarker` | `axes` | Size is `getScale().x()` |
| `ArrowMarker` | `arrow` | The pose is the arrow's center, matching the two-point constructor |
| `GeometryMarker` box, sphere | `box`, `sphere` | Always white: the class has no material |
| `GeometryMarker` cylinder | `cylinder` | Rotated +90 degrees about X, since three.js builds along Y and tesseract along Z |
| `GeometryMarker` capsule | `capsule` | Sent, but the current client only logs that capsules are not implemented |
| `ToolpathMarker` | `lines` and `axes` | |
| `ContactResultsMarker` | `arrow` | Colors and directions follow the Ignition visualization |
| `GeometryMarker` cone | `cone` | Rotated like a cylinder |
| `GeometryMarker` mesh and others; `LINE_*`, `POINTS`, `TEXT`, `TRIANGLE_*`, `USER_DEFINED` | none | Logged; the client has no equivalent, and tesseract has no classes for the enum-only types |

## Upstream issues found

- `TESSERACT_ADD_VISUALIZATION_PLUGIN` expands to `TESSERACT_ADD_PLUGIN_SECTIONED`, which is not defined at tesseract `2b79e91c`. The plugin calls `EXPORT_CLASS_SECTIONED` directly.
- `ArrowMarker`'s class comment puts the base at the pose, but its two-point constructor puts the middle there.

## Verification

Python reference outputs were not generated, because the aim is to remove Python. Parity rests on a line-by-line port, the gtest suites, and the browser suite, which runs the client in Chromium and compares the resulting three.js scene with tesseract's own forward kinematics. Screenshots are written to `build/browser`.

## Not done

- Python bindings and the `dependencies.rosinstall` entry (handoff step 5).
- Builds on Windows, macOS and arm64.

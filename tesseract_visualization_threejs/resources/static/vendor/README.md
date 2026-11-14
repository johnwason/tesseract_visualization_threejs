# Vendored client libraries

Served by the library so that the viewer works without internet access.

| Library | Version | License | Source |
| --- | --- | --- | --- |
| three.js | 0.153.0 | MIT, see `three/LICENSE` | https://unpkg.com/three@0.153.0/ |
| robust-websocket | 1.0.0 | ISC, see `robust-websocket/LICENSE` | https://www.npmjs.com/package/robust-websocket |

Only the modules the client imports, and their relative imports, are included.
`examples/jsm/environments/RoomEnvironment.js` is from https://github.com/mrdoob/three.js/blob/r153/examples/jsm/environments/RoomEnvironment.js.

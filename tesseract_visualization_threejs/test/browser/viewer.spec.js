// Runs the JavaScript client in Chromium against the C++ server and checks what three.js
// actually built. Expected poses come from tesseract's state solver, via the C++ fixture.
const { test, expect } = require('@playwright/test');
const { spawn } = require('child_process');
const path = require('path');
const readline = require('readline');

const OUT = process.env.VIEWER_BROWSER_OUTPUT || '/ws/build/browser';
const FIXTURE = process.env.VIEWER_BROWSER_FIXTURE || '/ws/build/colcon/tesseract_visualization_threejs/tesseract_visualization_threejs_browser_fixture';

class Fixture {
  async start() {
    this.proc = spawn(FIXTURE, [], { stdio: ['pipe', 'pipe', 'inherit'] });
    this.lines = readline.createInterface({ input: this.proc.stdout })[Symbol.asyncIterator]();
    this.port = JSON.parse((await this.lines.next()).value).port;
  }
  async send(command) {
    this.proc.stdin.write(command + '\n');
    const reply = JSON.parse((await this.lines.next()).value);
    if (!reply.ok) throw new Error(`Fixture failed on "${command}": ${reply.error}`);
    return reply;
  }
  stop() { this.proc.kill(); }
}

let fixture;

test.beforeAll(async () => {
  fixture = new Fixture();
  await fixture.start();
});
test.afterAll(() => fixture.stop());

/** @brief The viewer must work with no internet access */
async function blockExternalRequests(page, problems) {
  await page.route(url => url.protocol !== 'file:' && !['127.0.0.1', 'localhost'].includes(url.hostname), async route => {
    problems.externalRequests.push(route.request().url());
    await route.abort();
  });
}

/** @brief The viewer instance is private to the module. Only this test copy of app.js exposes it. */
async function exposeViewer(page) {
  await page.route('**/app.js', async route => {
    const response = await route.fetch();
    const source = await response.text();
    const hook = 'await viewer.createScene();';
    expect(source.split(hook).length, 'app.js bootstrap has changed').toBe(2);
    await route.fulfill({ response, body: source.replace(hook, `window.__viewer = viewer; ${hook}`) });
  });
}

/** @brief Opens the client and records anything that went wrong */
async function openViewer(page, { withoutWebsocket = false } = {}) {
  const problems = { pageErrors: [], badResponses: [], log: [], externalRequests: [] };
  page.on('pageerror', e => problems.pageErrors.push(String(e)));
  page.on('console', m => problems.log.push(m.text()));
  page.on('response', r => {
    const pathname = new URL(r.url()).pathname;
    const expected404 = ['/favicon.ico', '/tesseract_trajectory.json', '/tesseract_markers.json'].includes(pathname);
    if (r.status() >= 400 && !(r.status() === 404 && expected404)) problems.badResponses.push(`${r.status()} ${r.url()}`);
  });

  await blockExternalRequests(page, problems);

  await exposeViewer(page);
  // Accepted but never connected to the server, so no update message can reach the client
  if (withoutWebsocket) await page.routeWebSocket(/\/websocket$/, () => {});

  await page.goto(`http://127.0.0.1:${fixture.port}/`);
  await expect.poll(() => problems.log.some(l => l.includes('WebSocket connection established'))).toBeTruthy();
  await page.waitForFunction(() => window.__viewer && window.__viewer._root_env && window.__viewer._root_env.children.length > 0);
  return problems;
}

function expectNoProblems(problems) {
  expect(problems.pageErrors).toEqual([]);
  expect(problems.badResponses).toEqual([]);
  expect(problems.externalRequests).toEqual([]);
}

/** @brief The three.js scene, in tesseract's frame (undoing the client's Z-up root) */
function sceneSnapshot(page) {
  return page.evaluate(() => {
    const viewer = window.__viewer;
    const THREE = window.__three;
    const root = viewer._root_z_up;
    root.updateMatrixWorld(true);
    const inverse = root.matrixWorld.clone().invert();

    const describe = object => {
      const m = inverse.clone().multiply(object.matrixWorld);
      const position = new THREE.Vector3();
      const quaternion = new THREE.Quaternion();
      const scale = new THREE.Vector3();
      m.decompose(position, quaternion, scale);
      const box = new THREE.Box3();
      object.traverse(child => {
        if (!child.geometry) return;
        child.geometry.computeBoundingBox();
        if (child.geometry.boundingBox.isEmpty()) return;
        const local = child.geometry.boundingBox.clone().applyMatrix4(inverse.clone().multiply(child.matrixWorld));
        box.union(local);
      });
      return {
        position: position.toArray(),
        quaternion: quaternion.toArray(),
        boxMin: box.isEmpty() ? null : box.min.toArray(),
        boxMax: box.isEmpty() ? null : box.max.toArray(),
      };
    };

    const out = { links: {}, joints: {}, markers: {}, meshes: 0, triangles: 0, textures: [], colors: {}, shading: {} };
    viewer._root_env.traverse(object => {
      const data = object.userData || {};
      if (data.tesseract_link) out.links[data.tesseract_link.name] = describe(object);
      if (data.tesseract_joint) out.joints[data.tesseract_joint.name] = { ...describe(object), ...data.tesseract_joint };
      if (data.tesseract_marker) {
        out.markers[data.tesseract_marker] = { ...describe(object), type: object.type, tags: data.tesseract_tags };
        if (object.isLine2) out.markers[data.tesseract_marker].segments = object.geometry.attributes.instanceStart.count;
        const material = object.line ? object.line.material : object.material;
        if (material && material.color) out.markers[data.tesseract_marker].color = material.color.toArray();
      }
      if (object.isMesh && !data.tesseract_marker) {
        out.meshes += 1;
        const index = object.geometry.index;
        out.triangles += index ? index.count / 3 : 0;
        if (object.material.map && object.material.map.image) {
          const image = object.material.map.image;
          out.textures.push({ name: object.name, width: image.width, height: image.height });
        }
        out.colors[object.name] = object.material.color.toArray();
        out.shading[object.name] = { normals: !!object.geometry.attributes.normal, flat: object.material.flatShading };
      }
    });
    return out;
  });
}

/** @brief Gives the page access to THREE, using the same module the client imported */
async function exposeThree(page) {
  await page.evaluate(async () => {
    window.__three = await import('/vendor/three/build/three.module.js');
  });
}

function expectPose(actual, expected, label) {
  for (let i = 0; i < 3; ++i) expect(actual.position[i], `${label} position[${i}]`).toBeCloseTo(expected.position[i], 3);
  const dot = actual.quaternion.reduce((sum, v, i) => sum + v * expected.quaternion[i], 0);
  expect(Math.abs(dot), `${label} orientation`).toBeCloseTo(1, 4);
}

function expectSize(item, expected, label, digits = 2) {
  for (let i = 0; i < 3; ++i) expect(item.boxMax[i] - item.boxMin[i], `${label} size[${i}]`).toBeCloseTo(expected[i], digits);
}

async function shot(page, name) {
  await page.waitForTimeout(500);
  await page.screenshot({ path: `${OUT}/${name}.png` });
}

async function expectAllLinkPoses(page, poses) {
  await expect.poll(async () => {
    const scene = await sceneSnapshot(page);
    return Object.keys(poses).every(name => {
      const actual = scene.links[name];
      if (!actual) return false;
      const near = poses[name].position.every((v, i) => Math.abs(v - actual.position[i]) < 1e-3);
      const dot = actual.quaternion.reduce((s, v, i) => s + v * poses[name].quaternion[i], 0);
      return near && Math.abs(Math.abs(dot) - 1) < 1e-4;
    });
  }, { message: 'all link poses should match tesseract forward kinematics' }).toBeTruthy();
  const scene = await sceneSnapshot(page);
  for (const name of Object.keys(poses)) expectPose(scene.links[name], poses[name], name);
  return scene;
}

test('abb_irb2400 loads, renders, and matches forward kinematics at zero', async ({ page }) => {
  const env = await fixture.send('env abb');
  const problems = await openViewer(page);
  await exposeThree(page);

  const scene = await expectAllLinkPoses(page, env.poses);
  expect(Object.keys(scene.links).length).toBe(env.links);
  expect(scene.meshes).toBeGreaterThan(5);
  expect(scene.triangles).toBeGreaterThan(1000);
  await expect.poll(() => page.evaluate(() => window.__viewer._renderer.info.render.triangles)).toBeGreaterThan(1000);
  expect(scene.joints.joint_1.type).toBe(1);

  await shot(page, 'abb_zero');
  expectNoProblems(problems);
});

test('joint positions move every link to the forward kinematics pose', async ({ page }) => {
  await fixture.send('env abb');
  const problems = await openViewer(page);
  await exposeThree(page);

  const moved = await fixture.send('joints 0.3 -0.4 0.5 0.6 -0.7 0.8');
  await expectAllLinkPoses(page, moved.poses);
  await shot(page, 'abb_moved');

  const again = await fixture.send('joints -0.8 0.2 -0.3 1.0 0.4 -1.2');
  await expectAllLinkPoses(page, again.poses);
  expectNoProblems(problems);
});

test('a trajectory animates the joints', async ({ page }) => {
  await fixture.send('env abb');
  const problems = await openViewer(page);
  await exposeThree(page);

  await fixture.send('trajectory');
  const angleAt = () => page.evaluate(() => {
    let value = null;
    window.__viewer._root_env.traverse(o => {
      if (o.userData && o.userData.tesseract_joint && o.userData.tesseract_joint.name === 'joint_1') value = o.quaternion.toArray();
    });
    return value;
  });
  const samples = new Set();
  await expect.poll(async () => {
    samples.add(JSON.stringify(await angleAt()));
    return samples.size;
  }, { intervals: [100], message: 'joint_1 should keep moving' }).toBeGreaterThan(3);

  expect(await page.evaluate(() => window.__viewer._animation && window.__viewer._animation.duration)).toBeCloseTo(2, 3);
  expectNoProblems(problems);
});

test('primitive shapes have the URDF dimensions and colors', async ({ page }) => {
  const env = await fixture.send('env shapes');
  const problems = await openViewer(page);
  await exposeThree(page);
  await expectAllLinkPoses(page, env.poses);

  const scene = await sceneSnapshot(page);
  expectSize(scene.links.box_link, [0.6, 0.1, 0.2], 'box');
  expectSize(scene.links.sphere_link, [1.0, 1.0, 1.0], 'sphere', 1);
  // The cylinder's axis is Z in tesseract
  expectSize(scene.links.cylinder_link, [0.4, 0.4, 0.6], 'cylinder', 1);

  const color = name => scene.colors[Object.keys(scene.colors).find(k => k.includes(name))];
  expect(color('cylinder_link')[0]).toBeCloseTo(0.8, 5);
  expect(color('box_link')[1]).toBeCloseTo(0.8, 5);

  // three.js must receive the primitives' normals rather than deriving flat shading
  const shading = scene.shading[Object.keys(scene.shading).find(k => k.includes('sphere_link'))];
  expect(shading).toEqual({ normals: true, flat: false });
  await shot(page, 'shapes');
  expectNoProblems(problems);
});

test('textured meshes arrive with decoded images', async ({ page }) => {
  await fixture.send('env textured');
  const problems = await openViewer(page);
  await exposeThree(page);

  await expect.poll(async () => (await sceneSnapshot(page)).textures.length).toBeGreaterThan(0);
  for (const texture of (await sceneSnapshot(page)).textures) {
    expect(texture.width).toBeGreaterThan(0);
    expect(texture.height).toBeGreaterThan(0);
  }
  await shot(page, 'textured');
  expectNoProblems(problems);
});

test('PBR materials reach three.js with every texture slot', async ({ page }) => {
  await fixture.send('env pbr');
  const problems = await openViewer(page);
  await exposeThree(page);

  const materials = async () => page.evaluate(() => {
    const out = [];
    window.__viewer._root_env.traverse(object => {
      if (!object.isMesh || (object.userData || {}).tesseract_marker) return;
      const m = object.material;
      const map = t => (t ? { channel: t.channel, colorSpace: t.colorSpace, wrapS: t.wrapS, wrapT: t.wrapT,
                              magFilter: t.magFilter, loaded: !!(t.image && t.image.width > 0) } : null);
      out.push({
        name: object.name, transparent: m.transparent, depthWrite: m.depthWrite, side: m.side, alphaTest: m.alphaTest,
        opacity: m.opacity, metalness: m.metalness, roughness: m.roughness, emissive: m.emissive.toArray(),
        emissiveIntensity: m.emissiveIntensity, normalScale: m.normalScale ? m.normalScale.toArray() : null,
        aoMapIntensity: m.aoMapIntensity, map: map(m.map), metalnessMap: map(m.metalnessMap),
        roughnessMap: map(m.roughnessMap), normalMap: map(m.normalMap), aoMap: map(m.aoMap), emissiveMap: map(m.emissiveMap),
        normals: !!object.geometry.attributes.normal, uv: !!object.geometry.attributes.uv, uv2: !!object.geometry.attributes.uv1 || !!object.geometry.attributes.uv2,
      });
    });
    return out;
  });

  await expect.poll(async () => (await materials()).some(m => m.normalMap && m.normalMap.loaded && m.aoMap && m.aoMap.loaded)).toBe(true);
  const all = await materials();
  const full = all.find(m => m.normalMap);
  const mask = all.find(m => !m.normalMap && m.alphaTest > 0);
  expect(mask).toBeTruthy();

  const THREE = await page.evaluate(() => ({ DoubleSide: window.__three.DoubleSide,
    SRGB: window.__three.SRGBColorSpace, Clamp: window.__three.ClampToEdgeWrapping, Mirror: window.__three.MirroredRepeatWrapping,
    Nearest: window.__three.NearestFilter }));
  expect(full.transparent).toBe(true);
  expect(full.depthWrite).toBe(false);
  expect(full.side).toBe(THREE.DoubleSide);
  expect(full.opacity).toBeCloseTo(0.6, 5);
  expect(full.metalness).toBeCloseTo(0.75, 5);
  expect(full.roughness).toBeCloseTo(0.25, 5);
  expect(full.emissiveIntensity).toBeCloseTo(2.5, 5);
  expect(Math.abs(full.normalScale[0])).toBeCloseTo(0.5, 5);
  expect(full.aoMapIntensity).toBeCloseTo(0.8, 5);
  expect(full.normals).toBe(true);
  expect(full.uv && full.uv2).toBe(true);
  for (const slot of ['map', 'metalnessMap', 'roughnessMap', 'normalMap', 'aoMap', 'emissiveMap'])
    expect(full[slot] && full[slot].loaded, slot).toBe(true);
  expect(full.map.colorSpace).toBe(THREE.SRGB);
  expect(full.emissiveMap.colorSpace).toBe(THREE.SRGB);
  expect(full.normalMap.colorSpace).not.toBe(THREE.SRGB);
  expect(full.map.channel).toBe(0);
  expect(full.aoMap.channel).toBe(1);
  expect(full.map.wrapS).toBe(THREE.Clamp);
  expect(full.map.wrapT).toBe(THREE.Mirror);
  expect(full.map.magFilter).toBe(THREE.Nearest);

  expect(mask.alphaTest).toBeCloseTo(0.3, 5);
  expect(mask.transparent).toBe(false);
  // The client renders every material double sided, so meshes with inconsistent winding stay visible
  expect(mask.side).toBe(THREE.DoubleSide);
  await shot(page, 'pbr');
  expectNoProblems(problems);
});

test('every tesseract marker type reaches the scene correctly', async ({ page }) => {
  const env = await fixture.send('env shapes');
  const problems = await openViewer(page);
  await exposeThree(page);
  await expectAllLinkPoses(page, env.poses);

  const { names } = await fixture.send('markers');
  expect(names).toContain('cone');
  expect(names).toContain('capsule');

  // The marker after the capsule and the cone proves that the list was not cut short
  await expect.poll(async () => Object.keys((await sceneSnapshot(page)).markers)).toEqual(expect.arrayContaining(['after_unsupported', 'contact']));
  const { markers } = await sceneSnapshot(page);

  expect(markers.capsule).toBeUndefined();
  expect(problems.log.some(l => l.includes('capsule marker not implemented'))).toBeTruthy();

  expect(markers.axis.position[0]).toBeCloseTo(1, 5);
  expect(markers.axis.type).toBe('AxesHelper');

  // A box of 1 x 2 x 3 turned 90 degrees about Z, which checks the w, x, y, z convention
  expectSize(markers.box, [2, 1, 3], 'box marker');
  expect(markers.box.position[0]).toBeCloseTo(3, 5);

  expectSize(markers.sphere, [0.5, 0.5, 0.5], 'sphere marker', 1);

  // Must stand along tesseract's Z, not the Y axis three.js builds cylinders along
  expectSize(markers.cylinder, [0.2, 0.2, 0.4], 'cylinder marker', 1);

  // Like cylinders, cones must stand along tesseract's Z
  expectSize(markers.cone, [0.2, 0.2, 0.4], 'cone marker', 1);
  expect(markers.cone.position[0]).toBeCloseTo(5, 5);

  // The arrow goes from z = 1 to z = 2
  expect(markers.arrow.boxMin[2]).toBeCloseTo(1, 2);
  expect(markers.arrow.boxMax[2]).toBeCloseTo(2, 2);

  expect(markers.toolpath_0.segments).toBe(2);
  expect(markers.toolpath_1.type).toBe('AxesHelper');
  expect(markers.toolpath_3.position[1]).toBeCloseTo(1, 5);

  // A negative distance is drawn red. The arrow points at the selected link
  expect(markers.contact.boxMin[2]).toBeCloseTo(0, 2);
  expect(markers.contact.boxMax[2]).toBeCloseTo(1, 2);
  expect(markers.contact.color).toEqual([1, 0, 0]);
  expect(markers.arrow.color).toEqual([1, 0, 1]);
  expect(markers.toolpath_0.color).toEqual([0, 1, 0]);

  await shot(page, 'markers');
  expectNoProblems(problems);

  await fixture.send('clear');
  await expect.poll(async () => Object.keys((await sceneSnapshot(page)).markers).length).toBe(0);
  expectNoProblems(problems);
});

test('an environment change reloads the scene without a page reload', async ({ page }) => {
  await fixture.send('env shapes');
  const problems = await openViewer(page);
  await exposeThree(page);
  expect((await sceneSnapshot(page)).links.extra_link).toBeUndefined();

  const env = await fixture.send('env shapes_extra');
  await expect.poll(async () => (await sceneSnapshot(page)).links.extra_link !== undefined).toBeTruthy();
  await expectAllLinkPoses(page, env.poses);
  expectNoProblems(problems);
});

test('a page opened after the fact gets the current state', async ({ page }) => {
  await fixture.send('env shapes');
  const moved = await fixture.send('joints 0.5 0.25');
  await fixture.send('markers');

  const problems = await openViewer(page);
  await exposeThree(page);
  await expectAllLinkPoses(page, moved.poses);
  await expect.poll(async () => Object.keys((await sceneSnapshot(page)).markers).length).toBeGreaterThan(5);
  expectNoProblems(problems);
});

test('the GLB document loads and matches the glTF document', async ({ page }) => {
  await fixture.send('env abb');
  const problems = await openViewer(page);

  const summary = await page.evaluate(async () => {
    const { GLTFLoader } = await import('/vendor/three/examples/jsm/loaders/GLTFLoader.js');
    const describe = url => new Promise((resolve, reject) => {
      new GLTFLoader().load(url, gltf => {
        const nodes = [];
        let triangles = 0;
        gltf.scene.traverse(o => {
          nodes.push(o.name);
          if (o.geometry && o.geometry.index) triangles += o.geometry.index.count / 3;
        });
        resolve({ nodes: nodes.sort(), triangles });
      }, undefined, reject);
    });
    return { gltf: await describe('tesseract_scene.gltf'), glb: await describe('tesseract_scene.glb') };
  });

  expect(summary.glb.triangles).toBeGreaterThan(1000);
  expect(summary.glb).toEqual(summary.gltf);
  expectNoProblems(problems);
});

test('without websocket messages, the client finds changes by polling ETags', async ({ page }) => {
  await fixture.send('env shapes');
  const problems = await openViewer(page, { withoutWebsocket: true });
  await exposeThree(page);
  expect((await sceneSnapshot(page)).links.extra_link).toBeUndefined();

  await fixture.send('env_keep shapes_extra');
  await expect.poll(async () => (await sceneSnapshot(page)).links.extra_link !== undefined).toBeTruthy();

  // Markers are polled about every 10 seconds
  await fixture.send('markers');
  await expect.poll(async () => Object.keys((await sceneSnapshot(page)).markers), { timeout: 30000 }).toContain('after_unsupported');
  expectNoProblems(problems);
});

// A scene can change while markers and trajectories are still in place
const markerNames = async page => Object.keys((await sceneSnapshot(page)).markers);

test('markers and the trajectory survive a scene reload', async ({ page }) => {
  await fixture.send('env shapes');
  const problems = await openViewer(page);
  await exposeThree(page);

  await fixture.send('markers');
  await fixture.send('trajectory');
  await expect.poll(() => markerNames(page)).toEqual(expect.arrayContaining(['after_unsupported', 'cylinder']));

  await fixture.send('env_keep shapes_extra');
  await expect.poll(async () => (await sceneSnapshot(page)).links.extra_link !== undefined).toBeTruthy();

  await expect.poll(() => markerNames(page)).toEqual(expect.arrayContaining(['after_unsupported', 'cylinder']));
  const { markers } = await sceneSnapshot(page);
  expect(markers.sphere.position[1]).toBeCloseTo(3, 5);
  expectSize(markers.cylinder, [0.2, 0.2, 0.4], 'cylinder marker', 1);

  await expect.poll(() => page.evaluate(() => window.__viewer._animation && window.__viewer._animation.duration)).toBeCloseTo(2, 3);
  await expect.poll(() => page.evaluate(() => window.__viewer._animation_action && window.__viewer._animation_action.isRunning())).toBeTruthy();
  const quaternion = () => page.evaluate(() => {
    let value = null;
    window.__viewer._root_env.traverse(o => {
      if (o.userData && o.userData.tesseract_joint && o.userData.tesseract_joint.name === 'cylinder_joint') value = o.quaternion.toArray();
    });
    return JSON.stringify(value);
  });
  const samples = new Set();
  await expect.poll(async () => {
    samples.add(await quaternion());
    return samples.size;
  }, { intervals: [100], message: 'the trajectory should animate the reloaded joints' }).toBeGreaterThan(3);
  expectNoProblems(problems);
});

test('a marker on a removed link is dropped without losing the others', async ({ page }) => {
  await fixture.send('env shapes_extra');
  const problems = await openViewer(page);
  await exposeThree(page);

  await fixture.send('marker_on extra_link on_removed');
  await fixture.send('marker_on world on_world');
  await expect.poll(() => markerNames(page)).toEqual(expect.arrayContaining(['on_removed', 'on_world']));

  await fixture.send('env_keep shapes');
  await expect.poll(async () => (await sceneSnapshot(page)).links.extra_link === undefined).toBeTruthy();
  await expect.poll(() => markerNames(page)).toEqual(['on_world']);
  expect(problems.log.some(l => l.includes('Parent link not found: extra_link'))).toBeTruthy();
  expectNoProblems(problems);
});

test('a stored trajectory for removed joints does not break the new scene', async ({ page }) => {
  await fixture.send('env abb');
  const problems = await openViewer(page);
  await exposeThree(page);
  await fixture.send('trajectory');
  await expect.poll(() => page.evaluate(() => window.__viewer._animation && window.__viewer._animation.duration)).toBeCloseTo(2, 3);

  await fixture.send('env_keep shapes');
  await expect.poll(async () => (await sceneSnapshot(page)).links.box_link !== undefined).toBeTruthy();
  await page.waitForTimeout(1000);  // the client refetches the stored trajectory after a reload

  const moved = await fixture.send('joints 0.5 0.25');
  await expectAllLinkPoses(page, moved.poses);
  expectNoProblems(problems);
});

test('a trajectory for joints that do not exist does not throw', async ({ page }) => {
  await fixture.send('env shapes');
  const problems = await openViewer(page);
  await exposeThree(page);

  await fixture.send('stale_trajectory not_a_joint');
  await page.waitForTimeout(500);
  expectNoProblems(problems);

  const moved = await fixture.send('joints 0.5 0.25');
  await expectAllLinkPoses(page, moved.poses);
  expectNoProblems(problems);
});

test('a trajectory sent right after a scene change reaches the new scene', async ({ page }) => {
  await fixture.send('env abb');
  const problems = await openViewer(page);
  await exposeThree(page);

  // The trajectory arrives while the client still shows the old scene
  await fixture.send('env_keep shapes');
  await fixture.send('trajectory');

  await expect.poll(async () => (await sceneSnapshot(page)).links.box_link !== undefined).toBeTruthy();
  const samples = new Set();
  await expect.poll(async () => {
    samples.add(await page.evaluate(() => {
      let value = null;
      window.__viewer._root_env.traverse(o => {
        if (o.userData && o.userData.tesseract_joint && o.userData.tesseract_joint.name === 'cylinder_joint') value = o.quaternion.toArray();
      });
      return JSON.stringify(value);
    }));
    return samples.size;
  }, { intervals: [100], message: 'the trajectory should animate the new scene' }).toBeGreaterThan(3);
  expectNoProblems(problems);
});

test('textured meshes load again after the scene is replaced', async ({ page }) => {
  await fixture.send('env textured');
  const problems = await openViewer(page);
  await exposeThree(page);
  await expect.poll(async () => (await sceneSnapshot(page)).textures.length).toBeGreaterThan(0);
  const textures = (await sceneSnapshot(page)).textures.length;

  await fixture.send('env abb');
  await expect.poll(async () => (await sceneSnapshot(page)).links.link_1 !== undefined).toBeTruthy();
  const robot = await sceneSnapshot(page);
  expect(robot.textures.length).toBe(0);
  expect(robot.links.mesh_dae_link).toBeUndefined();
  expect(robot.triangles).toBeGreaterThan(1000);

  await fixture.send('env textured');
  await expect.poll(async () => (await sceneSnapshot(page)).textures.length).toBe(textures);
  for (const texture of (await sceneSnapshot(page)).textures) expect(texture.width).toBeGreaterThan(0);
  expect((await sceneSnapshot(page)).links.link_1).toBeUndefined();
  expectNoProblems(problems);
});

// The compiled examples, run as a user would run them
for (const [example, port, links] of [
  ['shapes_viewer', 18771, 4],
  ['material_mesh_viewer', 18772, 3],
  ['abb_irb2400_viewer', 18773, 8],
]) {
  test(`the ${example} example serves its scene`, async ({ page }) => {
    const binary = path.join(path.dirname(FIXTURE), `tesseract_visualization_threejs_${example}`);
    const proc = spawn(binary, [], {
      stdio: ['pipe', 'ignore', 'inherit'],
      env: { ...process.env, TESSERACT_VISUALIZATION_THREEJS_PORT: String(port) },
    });
    try {
      const problems = { pageErrors: [], badResponses: [], log: [], externalRequests: [] };
      page.on('pageerror', e => problems.pageErrors.push(String(e)));
      await blockExternalRequests(page, problems);
      await exposeViewer(page);
      await expect.poll(async () => {
        try {
          return (await page.request.get(`http://127.0.0.1:${port}/`)).status();
        } catch (e) {
          return 0;
        }
      }).toBe(200);

      await page.goto(`http://127.0.0.1:${port}/`);
      await page.waitForFunction(() => window.__viewer && window.__viewer._root_env && window.__viewer._root_env.children.length > 0);
      await exposeThree(page);
      await expect.poll(async () => Object.keys((await sceneSnapshot(page)).links).length).toBeGreaterThanOrEqual(links);

      if (example === 'abb_irb2400_viewer') {
        await expect.poll(async () => Object.keys((await sceneSnapshot(page)).markers).length).toBeGreaterThan(50);
        expect(await page.evaluate(() => window.__viewer._animation && window.__viewer._animation.duration)).toBeCloseTo(5, 3);
      }
      await shot(page, `example_${example}`);
      expect(problems.pageErrors).toEqual([]);
      expect(problems.externalRequests).toEqual([]);
    } finally {
      proc.kill();
    }
  });
}

// The pages in examples/javascript, embedding the viewer
const IFRAME_EXAMPLES = path.join(__dirname, '../../examples/javascript');

async function openIframeExample(page, file) {
  const problems = { pageErrors: [], badResponses: [], log: [], externalRequests: [] };
  page.on('pageerror', e => problems.pageErrors.push(String(e)));
  await blockExternalRequests(page, problems);
  await exposeViewer(page);
  await page.goto(`file://${IFRAME_EXAMPLES}/${file}?host=127.0.0.1&port=${fixture.port}`);
  await expect.poll(() => page.frames().length).toBeGreaterThan(1);
  const frame = page.frames().find(f => f.url().includes('index.html'));
  await frame.waitForFunction(() => window.__viewer && window.__viewer._root_env && window.__viewer._root_env.children.length > 0);
  return { frame, problems };
}

test('the iframe example drives the joints through postMessage', async ({ page }) => {
  await fixture.send('env abb');
  const { frame, problems } = await openIframeExample(page, 'iframe_joint_positions.html');

  const seen = new Set();
  await expect.poll(async () => {
    const z = await frame.evaluate(() => {
      let value = null;
      window.__viewer._root_env.traverse(o => {
        if (o.userData && o.userData.tesseract_joint && o.userData.tesseract_joint.name === 'joint_1') value = o.quaternion.z;
      });
      return value;
    });
    // joint_1 turns about Z, by 0.6 and then -0.6 radians
    if (Math.abs(Math.abs(z) - Math.sin(0.3)) < 1e-3) seen.add(Math.sign(z));
    return seen.size;
  }, { intervals: [100], message: 'joint_1 should reach both positions' }).toBe(2);

  expect(problems.pageErrors).toEqual([]);
  expect(problems.externalRequests).toEqual([]);
});

test('the iframe example sends a trajectory through postMessage', async ({ page }) => {
  await fixture.send('env abb');
  const { frame, problems } = await openIframeExample(page, 'iframe_trajectory.html');

  await expect.poll(() => frame.evaluate(() => window.__viewer._animation && window.__viewer._animation.duration)).toBeCloseTo(10, 3);
  expect(problems.pageErrors).toEqual([]);
  expect(problems.externalRequests).toEqual([]);
});

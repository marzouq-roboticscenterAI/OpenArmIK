/* SPDX-License-Identifier: Apache-2.0 */
(() => {
  'use strict';
  const JOINT_ORDER = ['openarm_left_joint1','openarm_left_joint2','openarm_left_joint3','openarm_left_joint4','openarm_left_joint5','openarm_left_joint6','openarm_left_joint7','openarm_right_joint1','openarm_right_joint2','openarm_right_joint3','openarm_right_joint4','openarm_right_joint5','openarm_right_joint6','openarm_right_joint7'];
  const MAX_WIDTH = 1920;
  const MAX_HEIGHT = 1080;
  const MAX_PIXELS = 1920 * 1080;
  const MAX_MESHES = 11;
  const MAX_INSTANCES = 23;
  const MAX_ENCODED_BYTES = 2498724;
  const MAX_TRIANGLES = 49956;
  const MAX_FILE_BYTES = 1139984;
  const MAX_FILE_TRIANGLES = 22798;
  const MAX_GPU_BYTES = MAX_TRIANGLES * 9 * 4;
  const MAX_CONTEXT_LOSSES = 2;
  const STALE_MS = 500;
  const PERIOD_MS = 1000 / 30;
  const canvas = document.getElementById('viewer-canvas');
  const overlay = document.getElementById('viewer-overlay');
  const metrics = document.getElementById('viewer-metrics');
  let gl, program, positionLocation, matrixLocation, colorLocation;
  let links = new Map(), roots = [], instances = [], meshBuffers = new Map();
  let positions = new Array(14).fill(0), acceptedSequence = -1n;
  let responseAt = 0, receiptAge = Infinity, stateFresh = false;
  let pollInFlight = false, nextPollDeadline = performance.now();
  let drawTimes = [], metricsRing = [], fences = [], rollbackResponses = 0;
  let contextLost = false, contextTerminal = false, visibilityOverride = null;
  let gpuBytes = 0, ready = false;
  let camera = {yaw: 0.72, pitch: -0.32, distance: 1.36};

  const identity = () => new Float32Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]);
  function multiply(a, b) {
    const out = new Float32Array(16);
    for (let column = 0; column < 4; ++column) {
      for (let row = 0; row < 4; ++row) {
        out[column * 4 + row] = a[row] * b[column * 4] +
          a[4 + row] * b[column * 4 + 1] +
          a[8 + row] * b[column * 4 + 2] +
          a[12 + row] * b[column * 4 + 3];
      }
    }
    return out;
  }
  function translation(value) {
    const out = identity();
    out[12] = value[0]; out[13] = value[1]; out[14] = value[2];
    return out;
  }
  function scale(value) {
    const out = identity();
    out[0] = value[0]; out[5] = value[1]; out[10] = value[2];
    return out;
  }
  function rpy(value) {
    const [roll,pitch,yaw] = value;
    const cr=Math.cos(roll), sr=Math.sin(roll), cp=Math.cos(pitch), sp=Math.sin(pitch);
    const cy=Math.cos(yaw), sy=Math.sin(yaw);
    return new Float32Array([
      cy*cp,sy*cp,-sp,0,
      cy*sp*sr-sy*cr,sy*sp*sr+cy*cr,cp*sr,0,
      cy*sp*cr+sy*sr,sy*sp*cr-cy*sr,cp*cr,0,
      0,0,0,1]);
  }
  function axisRotation(axis, angle) {
    const [x,y,z] = axis, c=Math.cos(angle), s=Math.sin(angle), d=1-c;
    return new Float32Array([
      c+x*x*d,y*x*d+z*s,z*x*d-y*s,0,
      x*y*d-z*s,c+y*y*d,z*y*d+x*s,0,
      x*z*d+y*s,y*z*d-x*s,c+z*z*d,0,
      0,0,0,1]);
  }
  function parseNumbers(value, fallback) {
    const parts = (value || fallback).trim().split(/\s+/).map(Number);
    if (parts.length !== 3 || parts.some(number => !Number.isFinite(number))) {
      throw new Error('nonfinite URDF vector');
    }
    return parts;
  }
  function finiteTransform(origin) {
    return multiply(
      translation(parseNumbers(origin.getAttribute('xyz'), '0 0 0')),
      rpy(parseNumbers(origin.getAttribute('rpy'), '0 0 0')));
  }
  function projection(aspect) {
    const f = 1 / Math.tan(0.55 / 2), near = .03, far = 10;
    const out = new Float32Array(16);
    out[0] = f / aspect; out[5] = f;
    out[10] = (far + near) / (near - far); out[11] = -1;
    out[14] = 2 * far * near / (near - far);
    return out;
  }
  const dot = (a,b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
  const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const normalize = value => {
    const norm = Math.hypot(value[0], value[1], value[2]);
    return norm > 0 ? [value[0]/norm,value[1]/norm,value[2]/norm] : [0,0,1];
  };
  function lookAt(eye, center) {
    const z = normalize([eye[0]-center[0],eye[1]-center[1],eye[2]-center[2]]);
    const x = normalize(cross([0,0,1],z)), y = cross(z,x), out = identity();
    out[0]=x[0];out[1]=y[0];out[2]=z[0];out[4]=x[1];out[5]=y[1];out[6]=z[1];
    out[8]=x[2];out[9]=y[2];out[10]=z[2];out[12]=-dot(x,eye);out[13]=-dot(y,eye);out[14]=-dot(z,eye);
    return out;
  }
  function assert(value, message) {if (!value) throw new Error(message);}
  function fail(message) {overlay.textContent = message; overlay.classList.remove('ok');}
  function setOverlay(message) {
    if (!message) {overlay.classList.add('ok'); return;}
    fail(message);
  }
  function compile(type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source); gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
      throw new Error(gl.getShaderInfoLog(shader) || 'shader compile failed');
    }
    return shader;
  }
  function initializeGl() {
    gl = canvas.getContext('webgl2', {alpha: false, antialias: true, depth: true});
    if (!gl) throw new Error('WEBGL2 UNAVAILABLE');
    const vertex = compile(gl.VERTEX_SHADER,
      '#version 300 es\nin vec3 aPosition;uniform mat4 uMatrix;void main(){gl_Position=uMatrix*vec4(aPosition,1.0);}');
    const fragment = compile(gl.FRAGMENT_SHADER,
      '#version 300 es\nprecision mediump float;uniform vec4 uColor;out vec4 color;void main(){color=uColor;}');
    program = gl.createProgram();
    gl.attachShader(program, vertex); gl.attachShader(program, fragment); gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      throw new Error(gl.getProgramInfoLog(program) || 'program link failed');
    }
    positionLocation = gl.getAttribLocation(program, 'aPosition');
    matrixLocation = gl.getUniformLocation(program, 'uMatrix');
    colorLocation = gl.getUniformLocation(program, 'uColor');
    gl.enable(gl.DEPTH_TEST); gl.disable(gl.CULL_FACE); gl.clearColor(.025,.04,.065,1);
    resize();
  }

  function binaryStl(buffer, expectedTriangles) {
    assert(buffer.byteLength >= 84 && buffer.byteLength <= MAX_FILE_BYTES, 'STL size outside cap');
    const view = new DataView(buffer), count = view.getUint32(80, true);
    assert(count === expectedTriangles && count <= MAX_FILE_TRIANGLES, 'STL triangle count mismatch');
    assert(buffer.byteLength === 84 + count * 50, 'STL length mismatch');
    const componentCount = count * 9;
    assert(componentCount * Float32Array.BYTES_PER_ELEMENT <= MAX_GPU_BYTES, 'STL decode cap exceeded');
    const vertices = new Float32Array(componentCount);
    let input = 84, output = 0;
    for (let triangle = 0; triangle < count; ++triangle) {
      input += 12;
      for (let vertex = 0; vertex < 3; ++vertex) {
        for (let axis = 0; axis < 3; ++axis) {
          const value = view.getFloat32(input + axis * 4, true);
          assert(Number.isFinite(value), 'nonfinite STL vertex');
          vertices[output++] = value;
        }
        input += 12;
      }
      input += 2;
    }
    return vertices;
  }
  async function fetchText(route, maximumBytes) {
    const response = await fetch(route, {cache: 'no-store', credentials: 'same-origin'});
    if (!response.ok) throw new Error('asset request failed: ' + route);
    const declared = Number(response.headers.get('Content-Length'));
    assert(Number.isInteger(declared) && declared > 0 && declared <= maximumBytes,
      'asset Content-Length outside cap: ' + route);
    const text = await response.text();
    assert(new TextEncoder().encode(text).byteLength === declared, 'asset length mismatch: ' + route);
    return text;
  }
  async function fetchJson(route, maximumBytes) {
    return JSON.parse(await fetchText(route, maximumBytes));
  }
  async function digestHex(buffer) {
    const digest = await crypto.subtle.digest('SHA-256', buffer);
    return Array.from(new Uint8Array(digest), value => value.toString(16).padStart(2, '0')).join('');
  }
  function validateManifest(manifest) {
    assert(manifest && manifest.schema === 1 && Array.isArray(manifest.meshes), 'invalid viewer manifest');
    assert(manifest.meshes.length === MAX_MESHES, 'unexpected viewer mesh count');
    assert(manifest.total_bytes === MAX_ENCODED_BYTES && manifest.total_triangles === MAX_TRIANGLES,
      'viewer aggregate declaration mismatch');
    const routes = new Set(), sources = new Set();
    let encodedBytes = 0, triangles = 0, decodedBytes = 0;
    for (const mesh of manifest.meshes) {
      assert(mesh && typeof mesh.route === 'string' && /^\/viewer\/mesh\/[A-Za-z0-9_.-]+\.stl$/.test(mesh.route),
        'invalid mesh route');
      assert(typeof mesh.source === 'string' && mesh.source.startsWith('package://openarm_description/'),
        'invalid mesh source');
      assert(!routes.has(mesh.route) && !sources.has(mesh.source), 'duplicate mesh route or source');
      assert(Number.isInteger(mesh.bytes) && mesh.bytes >= 84 && mesh.bytes <= MAX_FILE_BYTES,
        'mesh byte bound exceeded');
      assert(Number.isInteger(mesh.triangles) && mesh.triangles > 0 && mesh.triangles <= MAX_FILE_TRIANGLES,
        'mesh triangle bound exceeded');
      assert(mesh.bytes === 84 + mesh.triangles * 50, 'mesh encoded shape mismatch');
      assert(typeof mesh.sha256 === 'string' && /^[0-9a-f]{64}$/.test(mesh.sha256), 'invalid mesh digest');
      routes.add(mesh.route); sources.add(mesh.source);
      encodedBytes += mesh.bytes; triangles += mesh.triangles;
      decodedBytes += mesh.triangles * 9 * Float32Array.BYTES_PER_ELEMENT;
      assert(encodedBytes <= MAX_ENCODED_BYTES && triangles <= MAX_TRIANGLES && decodedBytes <= MAX_GPU_BYTES,
        'mesh aggregate cap exceeded');
    }
    assert(encodedBytes === MAX_ENCODED_BYTES && triangles === MAX_TRIANGLES && decodedBytes === MAX_GPU_BYTES,
      'mesh aggregate totals mismatch');
    return manifest.meshes;
  }
  async function loadMesh(mesh) {
    assert(meshBuffers.size < MAX_MESHES, 'GPU buffer count cap exceeded');
    const response = await fetch(mesh.route, {cache: 'no-store', credentials: 'same-origin'});
    if (!response.ok) throw new Error('mesh request failed');
    const declared = Number(response.headers.get('Content-Length'));
    assert(declared === mesh.bytes && declared <= MAX_FILE_BYTES, 'mesh Content-Length mismatch');
    const bytes = await response.arrayBuffer();
    assert(bytes.byteLength === mesh.bytes, 'mesh size mismatch');
    assert(await digestHex(bytes) === mesh.sha256, 'mesh SHA-256 mismatch');
    const vertices = binaryStl(bytes, mesh.triangles);
    assert(gpuBytes + vertices.byteLength <= MAX_GPU_BYTES, 'GPU byte cap exceeded');
    const buffer = gl.createBuffer();
    assert(buffer, 'GPU buffer allocation failed');
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer); gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.STATIC_DRAW);
    gpuBytes += vertices.byteLength;
    meshBuffers.set(mesh.source, {buffer, vertices: vertices.length / 3, bytes: vertices.byteLength});
  }
  function parseUrdf(text, manifest) {
    assert(!/<!doctype/i.test(text), 'doctype is not allowed');
    const documentNode = new DOMParser().parseFromString(text, 'application/xml');
    assert(!documentNode.querySelector('parsererror'), 'URDF parse error');
    assert(documentNode.documentElement.tagName === 'robot' && !documentNode.doctype, 'unexpected URDF root');
    const meshBySource = new Map(manifest.map(mesh => [mesh.source, mesh]));
    assert(meshBySource.size === MAX_MESHES, 'unexpected mesh allowlist');
    const direct = (element, tag) => Array.from(element.children).filter(child => child.tagName === tag);
    const one = (element, tag) => {const children = direct(element, tag); return children.length === 1 ? children[0] : null;};
    const linkElements = direct(documentNode.documentElement, 'link');
    const jointElements = direct(documentNode.documentElement, 'joint');
    assert(linkElements.length === 26 && jointElements.length === 25, 'unexpected Stage-A link or joint count');
    links = new Map();
    for (const element of linkElements) {
      const name = element.getAttribute('name');
      assert(name && !links.has(name), 'invalid link');
      links.set(name, {name, children: [], transform: identity(), meshes: []});
    }
    roots = [];
    const dynamic = [];
    for (const element of jointElements) {
      const name = element.getAttribute('name'), type = element.getAttribute('type');
      const parent = one(element, 'parent'), child = one(element, 'child'), origin = one(element, 'origin');
      assert(name && parent && child && origin && (type === 'fixed' || type === 'revolute'),
        'unsupported URDF joint');
      const parentLink = links.get(parent.getAttribute('link'));
      const childLink = links.get(child.getAttribute('link'));
      assert(parentLink && childLink, 'unknown joint link');
      const joint = {name, type, child: childLink, origin: finiteTransform(origin), axis: [0,0,1], position: -1};
      if (type === 'revolute') {
        const index = JOINT_ORDER.indexOf(name), axis = one(element, 'axis');
        assert(index >= 0 && axis, 'unexpected dynamic joint');
        joint.axis = parseNumbers(axis.getAttribute('xyz'), ''); joint.position = index; dynamic.push(name);
      }
      parentLink.children.push(joint); childLink.parented = true;
    }
    assert(dynamic.length === JOINT_ORDER.length && new Set(dynamic).size === JOINT_ORDER.length,
      'dynamic joint contract mismatch');
    for (const link of links.values()) if (!link.parented) roots.push(link);
    assert(roots.length === 1 && roots[0].name === 'world', 'unexpected root');
    instances = [];
    for (const element of linkElements) {
      const link = links.get(element.getAttribute('name'));
      for (const collision of direct(element, 'collision')) {
        assert(instances.length < MAX_INSTANCES, 'collision instance cap exceeded');
        const origin = one(collision, 'origin') || documentNode.createElement('origin');
        const geometry = one(collision, 'geometry'), mesh = geometry && one(geometry, 'mesh');
        assert(mesh, 'collision mesh missing');
        const source = mesh.getAttribute('filename');
        assert(meshBySource.has(source), 'mesh outside manifest');
        const instance = {
          source,
          transform: multiply(finiteTransform(origin), scale(parseNumbers(mesh.getAttribute('scale'), '1 1 1')))
        };
        link.meshes.push(instance); instances.push(instance);
      }
    }
    assert(instances.length === MAX_INSTANCES, 'unexpected collision instance count');
  }
  function updateTransforms(link, parent) {
    link.transform = parent;
    for (const joint of link.children) {
      let transform = multiply(parent, joint.origin);
      if (joint.position >= 0) transform = multiply(transform, axisRotation(joint.axis, positions[joint.position]));
      updateTransforms(joint.child, transform);
    }
  }
  function resize() {
    if (!gl) return;
    const rect = canvas.getBoundingClientRect();
    const ratio = Math.min(2, Math.max(1, window.devicePixelRatio || 1));
    const rawWidth = Math.min(MAX_WIDTH, Math.max(1, Math.floor(rect.width * ratio)));
    const rawHeight = Math.min(MAX_HEIGHT, Math.max(1, Math.floor(rect.height * ratio)));
    const area = rawWidth * rawHeight;
    const scaleFactor = Math.min(1, Math.sqrt(MAX_PIXELS / area));
    const width = Math.min(MAX_WIDTH, Math.max(1, Math.floor(rawWidth * scaleFactor)));
    const height = Math.min(MAX_HEIGHT, Math.max(1, Math.floor(rawHeight * scaleFactor)));
    assert(width * height <= MAX_PIXELS, 'canvas pixel cap exceeded');
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width; canvas.height = height; gl.viewport(0, 0, width, height);
    }
  }
  function cameraMatrix() {
    const cp = Math.cos(camera.pitch);
    const eye = [camera.distance*cp*Math.cos(camera.yaw), camera.distance*cp*Math.sin(camera.yaw),
      .42+camera.distance*Math.sin(camera.pitch)];
    return multiply(projection(canvas.width / canvas.height), lookAt(eye, [0,0,.42]));
  }
  function currentVisibility() {return visibilityOverride || document.visibilityState;}
  function viewAge(now) {return Number.isFinite(receiptAge) ? receiptAge + now - responseAt : Infinity;}
  function mark(name) {performance.mark(name); return name;}
  function recordMetric(record) {
    metricsRing.push(record);
    if (metricsRing.length > 512) {
      const removed = metricsRing.shift();
      if (removed.mark) performance.clearMarks(removed.mark);
    }
  }
  function pollFences() {
    while (fences.length) {
      const pending = fences[0], status = gl.clientWaitSync(pending.sync, 0, 0);
      if (status === gl.TIMEOUT_EXPIRED) break;
      gl.deleteSync(pending.sync); pending.metric.gpu_complete_time = performance.now(); fences.shift();
    }
  }
  function updateOverlay(now) {
    if (contextLost) return;
    if (currentVisibility() !== 'visible') {setOverlay('VIEW THROTTLED'); return;}
    const age = viewAge(now);
    setOverlay(!stateFresh || age > STALE_MS ? 'VIEW STALE' : '');
  }
  function visibilityChanged() {
    drawTimes = [];
    if (currentVisibility() !== 'visible') {
      metrics.textContent = 'VIEW THROTTLED · draw FPS paused · state age not live';
      setOverlay('VIEW THROTTLED');
    } else {
      metrics.textContent = 'draw FPS warming · state age refreshing';
      updateOverlay(performance.now());
    }
  }
  function draw(now) {
    requestAnimationFrame(draw);
    if (!gl || contextLost || currentVisibility() !== 'visible') return;
    resize(); updateTransforms(roots[0], identity());
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT); gl.useProgram(program);
    const vp = cameraMatrix(), stale = !stateFresh || viewAge(now) > STALE_MS;
    for (const link of links.values()) {
      for (const mesh of link.meshes) {
        const gpu = meshBuffers.get(mesh.source); if (!gpu) continue;
        gl.bindBuffer(gl.ARRAY_BUFFER, gpu.buffer); gl.enableVertexAttribArray(positionLocation);
        gl.vertexAttribPointer(positionLocation, 3, gl.FLOAT, false, 0, 0);
        gl.uniformMatrix4fv(matrixLocation, false, multiply(vp, multiply(link.transform, mesh.transform)));
        const right = link.name.includes('_right_');
        gl.uniform4f(colorLocation, right ? .3 : .2, right ? .62 : .42, .82, stale ? .28 : .92);
        gl.drawArrays(gl.TRIANGLES, 0, gpu.vertices);
      }
    }
    pollFences(); drawTimes.push(now);
    while (drawTimes.length && now - drawTimes[0] > 1000) drawTimes.shift();
    const metric = {sequence: acceptedSequence.toString(), receipt_age_ms: viewAge(now),
      client_receive_time: responseAt, draw_submit_time: now, visibility: currentVisibility(),
      context_lost: contextLost, mark: mark('openarm-viewer-draw-' + acceptedSequence.toString())};
    recordMetric(metric);
    const sync = gl.fenceSync(gl.SYNC_GPU_COMMANDS_COMPLETE, 0);
    if (sync) {
      fences.push({sync, metric});
      if (fences.length > 4) gl.deleteSync(fences.shift().sync);
      gl.flush();
    }
    metrics.textContent = 'draw FPS ' + drawTimes.length.toFixed(0) + ' · state age ' +
      (Number.isFinite(viewAge(now)) ? viewAge(now).toFixed(0) + ' ms' : 'unknown') +
      ' · sequence ' + (acceptedSequence < 0n ? '—' : acceptedSequence.toString()) +
      ' · ignored ' + rollbackResponses;
    updateOverlay(now);
  }
  function validViewState(value) {
    return value && value.schema === 1 && Array.isArray(value.joint_order) &&
      value.joint_order.length === JOINT_ORDER.length &&
      value.joint_order.every((name,index) => name === JOINT_ORDER[index]) &&
      typeof value.sequence === 'string' && /^\d+$/.test(value.sequence) &&
      typeof value.producer_time_ns === 'string' && /^-?\d+$/.test(value.producer_time_ns) &&
      typeof value.have_state === 'boolean' && typeof value.fresh === 'boolean';
  }
  function applyViewState(value, received = performance.now()) {
    if (!validViewState(value)) throw new Error('invalid view-state');
    if (!value.have_state) {
      stateFresh = false; responseAt = received; receiptAge = Infinity;
      return false;
    }
    const sequence = BigInt(value.sequence);
    if (sequence <= acceptedSequence) {
      rollbackResponses++;
      recordMetric({sequence: sequence.toString(), ignored: true, client_receive_time: received,
        visibility: currentVisibility(), context_lost: contextLost});
      return false;
    }
    assert(Array.isArray(value.position_rad) && value.position_rad.length === JOINT_ORDER.length &&
      value.position_rad.every(Number.isFinite), 'invalid joint positions');
    assert(Number.isFinite(value.receipt_age_ms) && value.receipt_age_ms >= 0, 'invalid receipt age');
    positions = value.position_rad.slice(); acceptedSequence = sequence;
    responseAt = received; receiptAge = value.receipt_age_ms; stateFresh = value.fresh;
    if (roots.length) updateTransforms(roots[0], identity());
    recordMetric({sequence: sequence.toString(), receipt_age_ms: receiptAge,
      client_receive_time: received, pose_apply_time: performance.now(), visibility: currentVisibility(),
      context_lost: contextLost, mark: mark('openarm-viewer-pose-' + sequence.toString())});
    return true;
  }
  async function pollViewState() {
    if (pollInFlight) return; pollInFlight = true;
    try {
      const response = await fetch('/api/view-state', {cache: 'no-store', credentials: 'same-origin'});
      const value = await response.json(), received = performance.now();
      if (!response.ok) throw new Error('view-state request failed');
      applyViewState(value, received);
    } catch (_) {
      stateFresh = false;
    } finally {
      pollInFlight = false;
      const now = performance.now(); nextPollDeadline += PERIOD_MS;
      while (nextPollDeadline <= now) nextPollDeadline += PERIOD_MS;
      window.setTimeout(pollViewState, Math.max(0, nextPollDeadline - now));
    }
  }
  function resetCamera() {camera = {yaw: .72, pitch: -.32, distance: 1.36};}
  function installCamera() {
    const active = new Map();
    let pinchDistance = null;
    const currentPinch = () => {
      const points = Array.from(active.values());
      return points.length < 2 ? null : Math.hypot(points[0][0]-points[1][0], points[0][1]-points[1][1]);
    };
    canvas.addEventListener('pointerdown', event => {
      event.preventDefault();
      try {canvas.setPointerCapture(event.pointerId);} catch (_) {}
      active.set(event.pointerId, [event.clientX,event.clientY]);
      pinchDistance = currentPinch();
    });
    canvas.addEventListener('pointermove', event => {
      if (!active.has(event.pointerId)) return;
      event.preventDefault();
      const previous = active.get(event.pointerId);
      active.set(event.pointerId, [event.clientX,event.clientY]);
      if (active.size >= 2) {
        const next = currentPinch();
        if (pinchDistance && next) {
          camera.distance = Math.max(.3, Math.min(3.0, camera.distance * pinchDistance / Math.max(1,next)));
        }
        pinchDistance = next;
        return;
      }
      pinchDistance = null;
      camera.yaw += (event.clientX-previous[0])*.009;
      camera.pitch = Math.max(-1.45,Math.min(1.45,camera.pitch+(event.clientY-previous[1])*.009));
    });
    const release = event => {
      active.delete(event.pointerId); pinchDistance = currentPinch();
    };
    canvas.addEventListener('pointerup', release);
    canvas.addEventListener('pointercancel', release);
    canvas.addEventListener('wheel', event => {
      event.preventDefault();
      camera.distance = Math.max(.3,Math.min(3.0,camera.distance*Math.exp(event.deltaY*.001)));
    }, {passive:false});
    document.getElementById('reset-view').addEventListener('click', resetCamera);
  }
  function contextLossCount() {
    try {return Number(sessionStorage.getItem('openarm-viewer-context-losses') || '0') || 0;}
    catch (_) {return MAX_CONTEXT_LOSSES + 1;}
  }
  function handleContextLost(event) {
    if (event) event.preventDefault();
    const count = contextLossCount() + 1;
    try {sessionStorage.setItem('openarm-viewer-context-losses', String(count));} catch (_) {}
    contextLost = true; contextTerminal = count > MAX_CONTEXT_LOSSES;
    drawTimes = [];
    metrics.textContent = contextTerminal ? 'WEBGL CONTEXT FAILED · reload disabled' :
      'WEBGL CONTEXT LOST · awaiting bounded restore';
    fail(contextTerminal ? 'WEBGL CONTEXT FAILED' : 'WEBGL CONTEXT LOST');
  }
  function handleContextRestored() {
    if (contextTerminal) {fail('WEBGL CONTEXT FAILED'); return;}
    window.location.reload();
  }
  function exposeTestHarness() {
    if (!navigator.webdriver) return;
    window.__openarmViewerTest = Object.freeze({
      ready: () => ready,
      setPositions: value => {
        assert(Array.isArray(value) && value.length === JOINT_ORDER.length && value.every(Number.isFinite),
          'test positions invalid');
        positions = value.slice(); if (roots.length) updateTransforms(roots[0], identity());
      },
      linkMatrix: name => Array.from(links.get(name).transform),
      meshMatrices: name => links.get(name).meshes.map(mesh => ({source: mesh.source,
        local: Array.from(mesh.transform), world: Array.from(multiply(links.get(name).transform, mesh.transform))})),
      camera: () => ({...camera}),
      state: () => ({sequence: acceptedSequence.toString(), rollbacks: rollbackResponses,
        fresh: stateFresh, age: viewAge(performance.now())}),
      applyViewState,
      updateOverlay: age => {
        responseAt = performance.now(); receiptAge = age; stateFresh = true;
        updateOverlay(performance.now()); return overlay.textContent;
      },
      setVisibility: value => {visibilityOverride = value; visibilityChanged();},
      overlay: () => overlay.textContent,
      metrics: () => metrics.textContent,
      context: () => ({lost: contextLost, terminal: contextTerminal, count: contextLossCount()}),
      setContextLossCount: value => sessionStorage.setItem('openarm-viewer-context-losses', String(value)),
      caps: () => ({gpuBytes, buffers: meshBuffers.size, instances: instances.length,
        canvasWidth: canvas.width, canvasHeight: canvas.height}),
      decodeStl: (buffer, expectedTriangles) => binaryStl(buffer, expectedTriangles)
    });
  }
  async function start() {
    try {
      initializeGl(); installCamera(); exposeTestHarness();
      new ResizeObserver(resize).observe(canvas);
      document.addEventListener('visibilitychange', visibilityChanged);
      const manifest = validateManifest(await fetchJson('/viewer/manifest.json', 16384));
      const urdf = await fetchText('/viewer/stage_a.urdf', 131072);
      parseUrdf(urdf, manifest);
      for (const mesh of manifest) await loadMesh(mesh);
      assert(meshBuffers.size === MAX_MESHES && gpuBytes === MAX_GPU_BYTES, 'GPU aggregate mismatch');
      ready = true; nextPollDeadline = performance.now(); pollViewState(); requestAnimationFrame(draw);
    } catch (error) {
      fail(error.message || 'VIEWER ERROR');
    }
  }
  canvas.addEventListener('webglcontextlost', handleContextLost);
  canvas.addEventListener('webglcontextrestored', handleContextRestored);
  exposeTestHarness();
  start();
})();

/* SPDX-License-Identifier: Apache-2.0 */
(() => {
  'use strict';
  const JOINT_ORDER = ['openarm_left_joint1','openarm_left_joint2','openarm_left_joint3','openarm_left_joint4','openarm_left_joint5','openarm_left_joint6','openarm_left_joint7','openarm_right_joint1','openarm_right_joint2','openarm_right_joint3','openarm_right_joint4','openarm_right_joint5','openarm_right_joint6','openarm_right_joint7'];
  const MAX_PIXELS = 1920 * 1080;
  const MAX_TRIANGLES = 49956;
  const STALE_MS = 500;
  const PERIOD_MS = 1000 / 30;
  const canvas = document.getElementById('viewer-canvas');
  const overlay = document.getElementById('viewer-overlay');
  const metrics = document.getElementById('viewer-metrics');
  let gl, program, positionLocation, matrixLocation, colorLocation;
  let links = new Map(), roots = [], instances = [], meshBuffers = new Map();
  let positions = new Array(14).fill(0), acceptedSequence = -1n, responseAt = 0, receiptAge = Infinity, stateFresh = false;
  let pollInFlight = false, nextPollDeadline = performance.now(), drawTimes = [], metricsRing = [], fences = [], rollbackResponses = 0;
  let contextLost = false, camera = {yaw: 0.72, pitch: -0.32, distance: 1.36};
  const identity = () => new Float32Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]);
  function multiply(a, b) { const out = new Float32Array(16); for (let c = 0; c < 4; ++c) for (let r = 0; r < 4; ++r) out[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3]; return out; }
  function translation(v) { const out = identity(); out[12] = v[0]; out[13] = v[1]; out[14] = v[2]; return out; }
  function scale(v) { const out = identity(); out[0] = v[0]; out[5] = v[1]; out[10] = v[2]; return out; }
  function rpy(v) { const [roll,pitch,yaw] = v, cr=Math.cos(roll), sr=Math.sin(roll), cp=Math.cos(pitch), sp=Math.sin(pitch), cy=Math.cos(yaw), sy=Math.sin(yaw); return new Float32Array([cy*cp,sy*cp,-sp,0,cy*sp*sr-sy*cr,sy*sp*sr+cy*cr,cp*sr,0,cy*sp*cr+sy*sr,sy*sp*cr-cy*sr,cp*cr,0,0,0,0,1]); }
  function axisRotation(axis, angle) { const [x,y,z] = axis, c=Math.cos(angle), s=Math.sin(angle), d=1-c; return new Float32Array([c+x*x*d,y*x*d+z*s,z*x*d-y*s,0,x*y*d-z*s,c+y*y*d,z*y*d+x*s,0,x*z*d+y*s,y*z*d-x*s,c+z*z*d,0,0,0,0,1]); }
  function parseNumbers(value, fallback) { const parts = (value || fallback).trim().split(/\s+/).map(Number); if (parts.length !== 3 || parts.some(value => !Number.isFinite(value))) throw new Error('nonfinite URDF vector'); return parts; }
  function finiteTransform(origin) { return multiply(translation(parseNumbers(origin.getAttribute('xyz'), '0 0 0')), rpy(parseNumbers(origin.getAttribute('rpy'), '0 0 0'))); }
  function projection(aspect) { const f = 1 / Math.tan(0.55 / 2), near = .03, far = 10, out = new Float32Array(16); out[0] = f / aspect; out[5] = f; out[10] = (far + near) / (near - far); out[11] = -1; out[14] = 2 * far * near / (near - far); return out; }
  function lookAt(eye, center) { const z = normalize([eye[0]-center[0],eye[1]-center[1],eye[2]-center[2]]), x = normalize(cross([0,0,1],z)), y = cross(z,x), out = identity(); out[0]=x[0];out[1]=y[0];out[2]=z[0];out[4]=x[1];out[5]=y[1];out[6]=z[1];out[8]=x[2];out[9]=y[2];out[10]=z[2];out[12]=-dot(x,eye);out[13]=-dot(y,eye);out[14]=-dot(z,eye); return out; }
  const dot = (a,b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
  const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const normalize = v => { const n = Math.hypot(v[0],v[1],v[2]); return n > 0 ? [v[0]/n,v[1]/n,v[2]/n] : [0,0,1]; };
  function fail(message) { overlay.textContent = message; overlay.classList.remove('ok'); }
  function setOverlay(message) { if (!message) {overlay.classList.add('ok'); return;} fail(message); }
  function assert(value, message) { if (!value) throw new Error(message); }
  function compile(type, source) { const shader = gl.createShader(type); gl.shaderSource(shader, source); gl.compileShader(shader); if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(shader) || 'shader compile failed'); return shader; }
  function initializeGl() {
    gl = canvas.getContext('webgl2', {alpha: false, antialias: true, depth: true});
    if (!gl) throw new Error('WEBGL2 UNAVAILABLE');
    const vertex = compile(gl.VERTEX_SHADER, '#version 300 es\nin vec3 aPosition;uniform mat4 uMatrix;void main(){gl_Position=uMatrix*vec4(aPosition,1.0);}');
    const fragment = compile(gl.FRAGMENT_SHADER, '#version 300 es\nprecision mediump float;uniform vec4 uColor;out vec4 color;void main(){color=uColor;}');
    program = gl.createProgram(); gl.attachShader(program, vertex); gl.attachShader(program, fragment); gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program) || 'program link failed');
    positionLocation = gl.getAttribLocation(program, 'aPosition'); matrixLocation = gl.getUniformLocation(program, 'uMatrix'); colorLocation = gl.getUniformLocation(program, 'uColor');
    gl.enable(gl.DEPTH_TEST); gl.disable(gl.CULL_FACE); gl.clearColor(.025,.04,.065,1); resize();
  }
  function binaryStl(buffer, expectedTriangles) {
    assert(buffer.byteLength >= 84, 'short STL'); const view = new DataView(buffer); const count = view.getUint32(80, true); assert(count === expectedTriangles && count <= MAX_TRIANGLES, 'STL triangle count mismatch'); assert(buffer.byteLength === 84 + count * 50, 'STL length mismatch');
    const vertices = new Float32Array(count * 9); let input = 84, output = 0;
    for (let triangle = 0; triangle < count; ++triangle) { input += 12; for (let vertex = 0; vertex < 3; ++vertex) { vertices[output++] = view.getFloat32(input, true); vertices[output++] = view.getFloat32(input + 4, true); vertices[output++] = view.getFloat32(input + 8, true); input += 12; } input += 2; }
    return vertices;
  }
  async function fetchJson(route) { const response = await fetch(route, {cache: 'no-store', credentials: 'same-origin'}); if (!response.ok) throw new Error('asset request failed: ' + route); return response.json(); }
  async function loadMesh(mesh) { const response = await fetch(mesh.route, {cache: 'no-store', credentials: 'same-origin'}); if (!response.ok) throw new Error('mesh request failed'); const bytes = await response.arrayBuffer(); assert(bytes.byteLength === mesh.bytes && bytes.byteLength <= 1200000, 'mesh size mismatch'); const vertices = binaryStl(bytes, mesh.triangles); const buffer = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, buffer); gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.STATIC_DRAW); meshBuffers.set(mesh.source, {buffer, vertices: vertices.length / 3}); }
  function parseUrdf(text, manifest) {
    assert(!/<!doctype/i.test(text), 'doctype is not allowed'); const documentNode = new DOMParser().parseFromString(text, 'application/xml'); assert(!documentNode.querySelector('parsererror'), 'URDF parse error'); assert(documentNode.documentElement.tagName === 'robot' && !documentNode.doctype, 'unexpected URDF root');
    const meshBySource = new Map(manifest.meshes.map(mesh => [mesh.source, mesh])); assert(meshBySource.size === 11, 'unexpected mesh allowlist');
    const direct = (element, tag) => Array.from(element.children).filter(child => child.tagName === tag);
    const one = (element, tag) => { const children = direct(element, tag); return children.length === 1 ? children[0] : null; };
    const linkElements = direct(documentNode.documentElement, 'link'); const jointElements = direct(documentNode.documentElement, 'joint'); assert(linkElements.length === 26 && jointElements.length === 25, 'unexpected Stage-A link or joint count');
    links = new Map(); for (const element of linkElements) { const name = element.getAttribute('name'); assert(name && !links.has(name), 'invalid link'); links.set(name, {name, children: [], transform: identity(), meshes: []}); }
    roots = []; const dynamic = [];
    for (const element of jointElements) {
      const name = element.getAttribute('name'), type = element.getAttribute('type'), parent = one(element, 'parent'), child = one(element, 'child'), origin = one(element, 'origin');
      assert(name && parent && child && origin && (type === 'fixed' || type === 'revolute'), 'unsupported URDF joint'); const parentLink = links.get(parent.getAttribute('link')), childLink = links.get(child.getAttribute('link')); assert(parentLink && childLink, 'unknown joint link');
      const joint = {name, type, child: childLink, origin: finiteTransform(origin), axis: [0,0,1], position: -1};
      if (type === 'revolute') { const index = JOINT_ORDER.indexOf(name); assert(index >= 0, 'unexpected dynamic joint'); const axis = one(element, 'axis'); assert(axis, 'dynamic joint axis missing'); joint.axis = parseNumbers(axis.getAttribute('xyz'), ''); joint.position = index; dynamic.push(name); }
      parentLink.children.push(joint); childLink.parented = true;
    }
    assert(dynamic.length === 14 && new Set(dynamic).size === 14, 'dynamic joint contract mismatch'); for (const link of links.values()) if (!link.parented) roots.push(link); assert(roots.length === 1 && roots[0].name === 'world', 'unexpected root');
    instances = []; for (const element of linkElements) { const link = links.get(element.getAttribute('name')); for (const collision of direct(element, 'collision')) { const origin = one(collision, 'origin') || documentNode.createElement('origin'); const geometry = one(collision, 'geometry'); const mesh = geometry && one(geometry, 'mesh'); assert(mesh, 'collision mesh missing'); const source = mesh.getAttribute('filename'); assert(meshBySource.has(source), 'mesh outside manifest'); link.meshes.push({source, transform: multiply(finiteTransform(origin), scale(parseNumbers(mesh.getAttribute('scale'), '1 1 1')))}); instances.push(link.meshes[link.meshes.length - 1]); } }
    assert(instances.length === 23, 'unexpected collision instance count'); return manifest.meshes;
  }
  function updateTransforms(link, parent) { link.transform = parent; for (const joint of link.children) { let transform = multiply(parent, joint.origin); if (joint.position >= 0) transform = multiply(transform, axisRotation(joint.axis, positions[joint.position])); updateTransforms(joint.child, transform); } }
  function resize() { if (!gl) return; const rect = canvas.getBoundingClientRect(), rawWidth = Math.max(1, Math.floor(rect.width * Math.min(2, window.devicePixelRatio || 1))), rawHeight = Math.max(1, Math.floor(rect.height * Math.min(2, window.devicePixelRatio || 1))), scaleFactor = Math.min(1, Math.sqrt(MAX_PIXELS / (rawWidth * rawHeight))), width = Math.max(1, Math.floor(rawWidth * scaleFactor)), height = Math.max(1, Math.floor(rawHeight * scaleFactor)); if (canvas.width !== width || canvas.height !== height) {canvas.width = width; canvas.height = height; gl.viewport(0, 0, width, height);} }
  function cameraMatrix() { const cp = Math.cos(camera.pitch), eye = [camera.distance * cp * Math.cos(camera.yaw), camera.distance * cp * Math.sin(camera.yaw), .42 + camera.distance * Math.sin(camera.pitch)]; return multiply(projection(canvas.width / canvas.height), lookAt(eye, [0,0,.42])); }
  function viewAge(now) { return Number.isFinite(receiptAge) ? receiptAge + now - responseAt : Infinity; }
  function recordMetric(record) { metricsRing.push(record); if (metricsRing.length > 512) { const removed = metricsRing.shift(); if (removed.mark) performance.clearMarks(removed.mark); } }
  function mark(name) { performance.mark(name); return name; }
  function pollFences() { while (fences.length) { const pending = fences[0], status = gl.clientWaitSync(pending.sync, 0, 0); if (status === gl.TIMEOUT_EXPIRED) break; gl.deleteSync(pending.sync); pending.metric.gpu_complete_time = performance.now(); fences.shift(); } }
  function updateOverlay(now) { if (contextLost) return; if (document.visibilityState !== 'visible') {setOverlay('VIEW THROTTLED'); return;} const age = viewAge(now); if (!stateFresh || age > STALE_MS) setOverlay('VIEW STALE'); else setOverlay(''); }
  function draw(now) {
    requestAnimationFrame(draw); if (!gl || contextLost) return; resize(); updateTransforms(roots[0], identity()); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT); gl.useProgram(program); const vp = cameraMatrix(), stale = !stateFresh || viewAge(now) > STALE_MS;
    for (const link of links.values()) for (const mesh of link.meshes) { const gpu = meshBuffers.get(mesh.source); if (!gpu) continue; gl.bindBuffer(gl.ARRAY_BUFFER, gpu.buffer); gl.enableVertexAttribArray(positionLocation); gl.vertexAttribPointer(positionLocation, 3, gl.FLOAT, false, 0, 0); gl.uniformMatrix4fv(matrixLocation, false, multiply(vp, multiply(link.transform, mesh.transform))); const right = link.name.includes('_right_'); gl.uniform4f(colorLocation, right ? .3 : .2, right ? .62 : .42, .82, stale ? .28 : .92); gl.drawArrays(gl.TRIANGLES, 0, gpu.vertices); }
    pollFences(); drawTimes.push(now); while (drawTimes.length && now - drawTimes[0] > 1000) drawTimes.shift(); const metric = {sequence: acceptedSequence.toString(), receipt_age_ms: viewAge(now), client_receive_time: responseAt, draw_submit_time: now, visibility: document.visibilityState, context_lost: contextLost, mark: mark('openarm-viewer-draw-' + acceptedSequence.toString())}; recordMetric(metric); const sync = gl.fenceSync(gl.SYNC_GPU_COMMANDS_COMPLETE, 0); if (sync) {fences.push({sync, metric}); if (fences.length > 4) {gl.deleteSync(fences.shift().sync);} gl.flush();} metrics.textContent = 'draw FPS ' + drawTimes.length.toFixed(0) + ' · state age ' + (Number.isFinite(viewAge(now)) ? viewAge(now).toFixed(0) + ' ms' : 'unknown') + ' · sequence ' + (acceptedSequence < 0n ? '—' : acceptedSequence.toString()) + ' · ignored ' + rollbackResponses; updateOverlay(now);
  }
  function validViewState(value) { return value && value.schema === 1 && Array.isArray(value.joint_order) && value.joint_order.length === JOINT_ORDER.length && value.joint_order.every((name,index) => name === JOINT_ORDER[index]) && typeof value.sequence === 'string' && typeof value.producer_time_ns === 'string' && typeof value.have_state === 'boolean' && typeof value.fresh === 'boolean'; }
  async function pollViewState() {
    if (pollInFlight) return; pollInFlight = true;
    try { const response = await fetch('/api/view-state', {cache: 'no-store', credentials: 'same-origin'}); const value = await response.json(); const received = performance.now(); if (!response.ok || !validViewState(value)) throw new Error('invalid view-state'); if (!value.have_state) {stateFresh = false; responseAt = received; receiptAge = Infinity; return;} const sequence = BigInt(value.sequence); if (sequence <= acceptedSequence) {rollbackResponses++; recordMetric({sequence: sequence.toString(), ignored: true, client_receive_time: received, visibility: document.visibilityState, context_lost: contextLost}); return;} assert(Array.isArray(value.position_rad) && value.position_rad.length === JOINT_ORDER.length && value.position_rad.every(Number.isFinite), 'invalid joint positions'); assert(Number.isFinite(value.receipt_age_ms) && value.receipt_age_ms >= 0, 'invalid receipt age'); positions = value.position_rad.slice(); acceptedSequence = sequence; responseAt = received; receiptAge = value.receipt_age_ms; stateFresh = value.fresh; recordMetric({sequence: sequence.toString(), receipt_age_ms: receiptAge, client_receive_time: received, pose_apply_time: performance.now(), visibility: document.visibilityState, context_lost: contextLost, mark: mark('openarm-viewer-pose-' + sequence.toString())}); } catch (_) {stateFresh = false;} finally { pollInFlight = false; const now = performance.now(); nextPollDeadline += PERIOD_MS; while (nextPollDeadline <= now) nextPollDeadline += PERIOD_MS; window.setTimeout(pollViewState, Math.max(0, nextPollDeadline - now)); }
  }
  function installCamera() {
    let pointer = null, last = null, pinchDistance = null;
    canvas.addEventListener('pointerdown', event => {canvas.setPointerCapture(event.pointerId); pointer = event.pointerId; last = [event.clientX,event.clientY];});
    canvas.addEventListener('pointermove', event => {if (event.pointerId !== pointer || !last) return; camera.yaw += (event.clientX-last[0])*.009; camera.pitch = Math.max(-1.45,Math.min(1.45,camera.pitch+(event.clientY-last[1])*.009)); last=[event.clientX,event.clientY];});
    canvas.addEventListener('pointerup', event => {if (event.pointerId === pointer) {pointer=null;last=null;}}); canvas.addEventListener('pointercancel', () => {pointer=null;last=null;});
    canvas.addEventListener('wheel', event => {event.preventDefault(); camera.distance = Math.max(.3,Math.min(3.0,camera.distance*Math.exp(event.deltaY*.001)));}, {passive:false});
    canvas.addEventListener('touchstart', event => {if(event.touches.length===2) pinchDistance=Math.hypot(event.touches[0].clientX-event.touches[1].clientX,event.touches[0].clientY-event.touches[1].clientY);}, {passive:true});
    canvas.addEventListener('touchmove', event => {if(event.touches.length!==2 || !pinchDistance) return; const next=Math.hypot(event.touches[0].clientX-event.touches[1].clientX,event.touches[0].clientY-event.touches[1].clientY); camera.distance=Math.max(.3,Math.min(3.0,camera.distance*pinchDistance/Math.max(1,next))); pinchDistance=next;}, {passive:true});
    canvas.addEventListener('touchend', () => {pinchDistance=null;}, {passive:true}); document.getElementById('reset-view').addEventListener('click', () => {camera={yaw:.72,pitch:-.32,distance:1.36};});
  }
  async function start() {
    try { initializeGl(); installCamera(); new ResizeObserver(resize).observe(canvas); const manifest = await fetchJson('/viewer/manifest.json'); assert(manifest.schema === 1 && Array.isArray(manifest.meshes) && manifest.meshes.length === 11 && manifest.total_triangles === MAX_TRIANGLES, 'invalid viewer manifest'); const response = await fetch('/viewer/stage_a.urdf', {cache:'no-store', credentials:'same-origin'}); if (!response.ok) throw new Error('URDF request failed'); const meshes = parseUrdf(await response.text(), manifest); await Promise.all(meshes.map(loadMesh)); nextPollDeadline = performance.now(); pollViewState(); requestAnimationFrame(draw); } catch (error) {fail(error.message || 'VIEWER ERROR');}
  }
  canvas.addEventListener('webglcontextlost', event => {event.preventDefault(); contextLost = true; fail('WEBGL CONTEXT LOST');}); canvas.addEventListener('webglcontextrestored', () => {window.location.reload();});
  start();
})();

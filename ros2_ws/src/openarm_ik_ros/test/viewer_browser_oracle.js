const done = arguments[arguments.length - 1];
try {
  const test = window.__openarmViewerTest;
  if (!test || !test.ready()) throw new Error('viewer test harness is not ready');
  const close = (actual, expected, tolerance = 2e-5) => {
    if (actual.length !== expected.length || actual.some((value, index) =>
      !Number.isFinite(value) || Math.abs(value - expected[index]) > tolerance)) {
      throw new Error('matrix oracle mismatch: ' + JSON.stringify(actual));
    }
  };
  const targets = JSON.parse(document.getElementById('portal-targets').textContent);
  const presetButtons = Array.from(document.querySelectorAll('button.preset'))
    .filter(button => button.dataset.preset !== 'current');
  if (presetButtons.length !== 18) throw new Error('expected exactly 18 dynamic target buttons');
  const rendered = new Set();
  for (const button of presetButtons) {
    const side = button.dataset.side, id = button.dataset.preset;
    const key = side + ':' + id;
    if (rendered.has(key)) throw new Error('duplicate target button ' + key);
    rendered.add(key);
    const entry = targets[side].find(target => target.id === id);
    if (!entry || button.textContent !== entry.label) throw new Error('target label/data mismatch ' + key);
    button.click();
    const prefix = side === 'left' ? 'l' : 'r';
    ['x','y','z'].forEach((axis, index) => {
      const expected = (entry.point[index] * 100).toFixed(4);
      if (document.getElementById(prefix + axis).value !== expected) {
        throw new Error('target field rendering mismatch ' + key + ':' + axis);
      }
    });
  }
  const selectUnit = value => {
    const radio = document.querySelector('input[name="coordinate-unit"][value="' + value + '"]');
    if (!radio) throw new Error('missing coordinate unit ' + value);
    radio.checked = true;
    radio.dispatchEvent(new Event('change', {bubbles:true}));
  };
  const lastLeft = targets.left[targets.left.length - 1].point;
  const lastRight = targets.right[targets.right.length - 1].point;
  const assertFields = (side, point, scale, digits) => {
    const prefix = side === 'left' ? 'l' : 'r';
    ['x','y','z'].forEach((axis, index) => {
      const expected = (point[index] * scale).toFixed(digits);
      if (document.getElementById(prefix + axis).value !== expected) {
        throw new Error('coordinate unit rendering mismatch ' + side + ':' + axis);
      }
    });
  };
  selectUnit('m');
  assertFields('left', lastLeft, 1, 6);
  assertFields('right', lastRight, 1, 6);
  selectUnit('in');
  assertFields('left', lastLeft, 1 / 0.0254, 6);
  assertFields('right', lastRight, 1 / 0.0254, 6);
  selectUnit('cm');
  assertFields('left', lastLeft, 100, 4);
  assertFields('right', lastRight, 100, 4);
  const motionSlider = document.getElementById('motion-limit-scale');
  if (motionSlider.value !== '80' || document.getElementById('motion-limit-value').textContent !== '80%') {
    throw new Error('movement limit default mismatch');
  }
  motionSlider.value = '100';
  motionSlider.dispatchEvent(new Event('input', {bubbles:true}));
  if (document.getElementById('motion-limit-value').textContent !== '100%') {
    throw new Error('movement limit label did not update');
  }
  motionSlider.value = '80';
  motionSlider.dispatchEvent(new Event('input', {bubbles:true}));

  const observed = [];
  let capturedMove = null;
  const originalFetch = window.fetch;
  window.fetch = function(resource, options) {
    const request = {url: String(resource), method: String(options && options.method || 'GET').toUpperCase()};
    observed.push(request);
    if (request.url === '/api/v3/move' && request.method === 'POST') {
      capturedMove = JSON.parse(options.body);
      return Promise.resolve({ok:true, json:() => Promise.resolve({message:'oracle move', projected:false,
        achieved_fraction:1, motion_limit_scale:capturedMove.motion_limit_scale})});
    }
    return originalFetch.apply(this, arguments);
  };
  const canvas = document.getElementById('viewer-canvas');
  const pointer = (type, id, x, y, pointerType = 'touch') => canvas.dispatchEvent(
    new PointerEvent(type, {pointerId:id, clientX:x, clientY:y, pointerType,
      bubbles:true, cancelable:true, isPrimary:id === 1}));

  document.getElementById('reset-view').click();
  const reset = test.camera();
  const paletteButton = document.getElementById('viewer-neutral-palette');
  const initialPalette = test.palette(), initialCaps = test.caps();
  if (initialPalette.name !== 'blue' || paletteButton.getAttribute('aria-pressed') !== 'false') {
    throw new Error('blue palette was not the accessible default');
  }
  paletteButton.click();
  const neutralPalette = test.palette(), neutralCaps = test.caps();
  if (neutralPalette.name !== 'neutral' || paletteButton.getAttribute('aria-pressed') !== 'true' ||
      neutralPalette.background.every((value, index) => value === initialPalette.background[index])) {
    throw new Error('neutral palette toggle failed');
  }
  if (JSON.stringify(neutralCaps) !== JSON.stringify(initialCaps)) {
    throw new Error('palette toggle changed viewer resource caps');
  }
  paletteButton.click();
  if (test.palette().name !== 'blue' || paletteButton.getAttribute('aria-pressed') !== 'false') {
    throw new Error('blue palette restore failed');
  }
  pointer('pointerdown', 1, 100, 100, 'mouse');
  pointer('pointermove', 1, 130, 118, 'mouse');
  pointer('pointerup', 1, 130, 118, 'mouse');
  const orbited = test.camera();
  if (orbited.yaw === reset.yaw || orbited.pitch === reset.pitch) throw new Error('one-pointer orbit failed');
  const beforeWheel = orbited.distance;
  canvas.dispatchEvent(new WheelEvent('wheel', {deltaY:120, bubbles:true, cancelable:true}));
  if (test.camera().distance === beforeWheel) throw new Error('wheel zoom failed');

  document.getElementById('reset-view').click();
  pointer('pointerdown', 10, 100, 100);
  pointer('pointerdown', 11, 200, 100);
  const beforePinch = test.camera();
  pointer('pointermove', 11, 250, 100);
  const afterPinch = test.camera();
  if (afterPinch.distance === beforePinch.distance) throw new Error('pinch zoom failed');
  if (afterPinch.yaw !== beforePinch.yaw || afterPinch.pitch !== beforePinch.pitch) {
    throw new Error('pinch incorrectly orbited');
  }
  pointer('pointercancel', 10, 100, 100);
  pointer('pointercancel', 11, 250, 100);
  document.getElementById('reset-view').click();
  const afterReset = test.camera();
  if (afterReset.yaw !== reset.yaw || afterReset.pitch !== reset.pitch ||
      afterReset.distance !== reset.distance) throw new Error('camera reset failed');
  if (observed.some(request => request.method === 'POST')) {
    throw new Error('camera/preset event emitted a POST: ' + JSON.stringify(observed));
  }
  document.getElementById('left').disabled = false;
  document.getElementById('left').click();
  if (!capturedMove || capturedMove.side !== 'left' || capturedMove.unit !== 'cm' ||
      capturedMove.motion_limit_scale !== 0.8 ||
      Math.abs(capturedMove.x - lastLeft[0] * 100) > 1e-9 ||
      Math.abs(capturedMove.y - lastLeft[1] * 100) > 1e-9 ||
      Math.abs(capturedMove.z - lastLeft[2] * 100) > 1e-9) {
    throw new Error('v3 movement payload mismatch: ' + JSON.stringify(capturedMove));
  }

  const order = ['openarm_left_joint1','openarm_left_joint2','openarm_left_joint3','openarm_left_joint4','openarm_left_joint5','openarm_left_joint6','openarm_left_joint7','openarm_right_joint1','openarm_right_joint2','openarm_right_joint3','openarm_right_joint4','openarm_right_joint5','openarm_right_joint6','openarm_right_joint7'];
  const asymmetric = [.11,-.22,.33,-.44,.55,-.66,.77,-.17,.28,-.39,.46,-.58,.69,-.73];
  const state = (sequence, position, age = 1, fresh = true) => ({schema:1, have_state:true,
    fresh, sequence:String(sequence), producer_time_ns:'1', receipt_age_ms:age,
    joint_order:order, position_rad:position});
  // Stay ahead of the live 30 Hz poll sequence instead of assuming it is below
  // a fixed test value by the time Firefox executes this asynchronous oracle.
  const oracleSequence = BigInt(test.state().sequence) + 100n;
  if (!test.applyViewState(state(oracleSequence, asymmetric))) throw new Error('fresh pose was not applied');
  if (test.applyViewState(state(oracleSequence - 1n, new Array(14).fill(1)))) {
    throw new Error('rollback was applied');
  }
  const afterRollback = test.state();
  if (afterRollback.sequence !== oracleSequence.toString() || afterRollback.rollbacks < 1) {
    throw new Error('rollback accounting failed');
  }
  if (test.updateOverlay(700) !== 'VIEW STALE') throw new Error('stale overlay failed');
  test.setVisibility('hidden');
  if (test.overlay() !== 'VIEW THROTTLED' || !test.metrics().includes('FPS paused')) {
    throw new Error('hidden-tab throttle state failed');
  }
  test.setVisibility('visible');

  const left7 = test.linkMatrix('openarm_left_link7');
  const right7 = test.linkMatrix('openarm_right_link7');
  const leftFinger = test.meshMatrices('openarm_left_left_finger')[0];
  const rightFinger = test.meshMatrices('openarm_right_right_finger')[0];
  // Constants are generated from the independently checked Stage-A URDF/TF transform oracle.
  close(left7, [-0.2161366642,-0.5748510957,-0.7891964316,0,
    -0.2587691844,-0.7456699610,0.6140152812,0,
    -0.9414475560,0.3369309008,0.0124128778,0,
    -0.1303039938,0.2732512355,0.3110575974,1], 2e-6);
  close(right7, [0.1511632949,0.8105254769,-0.5658605695,0,
    0.7984836102,-0.4375739694,-0.4134647548,0,
    -0.5827295184,-0.3893296719,-0.7133364081,0,
    0.0184688978,-0.2327470928,0.2813513875,1], 2e-6);
  close(leftFinger.world, [-0.0002161367,-0.0005748511,-0.0007891965,0,
    -0.0002587692,-0.0007456700,0.0006140153,0,
    -0.0009414476,0.0003369309,0.0000124129,0,
    0.4071400464,0.1186301410,0.2764943242,1], 2e-6);
  close(rightFinger.world, [0.0001511633,0.0008105255,-0.0005658606,0,
    -0.0007984837,0.0004375740,0.0004134648,0,
    -0.0005827295,-0.0003893297,-0.0007133365,0,
    0.3798556924,-0.0349969082,0.6611446142,1], 2e-6);
  if (!(leftFinger.local[0] > 0 && leftFinger.local[5] > 0 &&
        rightFinger.local[0] > 0 && rightFinger.local[5] < 0)) {
    throw new Error('mirrored finger mesh scale was not retained');
  }
  const caps = test.caps();
  if (caps.buffers !== 11 || caps.instances !== 23 || caps.gpuBytes !== 49956*9*4 ||
      caps.canvasWidth > 1920 || caps.canvasHeight > 1080 || caps.canvasWidth*caps.canvasHeight > 1920*1080) {
    throw new Error('viewer resource caps failed: ' + JSON.stringify(caps));
  }
  const malformedStl = new ArrayBuffer(134), malformedView = new DataView(malformedStl);
  malformedView.setUint32(80, 1, true);
  malformedView.setFloat32(96, Number.NaN, true);
  let rejectedNonfinite = false;
  try {test.decodeStl(malformedStl, 1);} catch (_) {rejectedNonfinite = true;}
  if (!rejectedNonfinite) throw new Error('nonfinite STL vertex was accepted');

  test.setContextLossCount(2);
  canvas.dispatchEvent(new Event('webglcontextlost', {cancelable:true}));
  const context = test.context();
  if (!context.lost || !context.terminal || context.count !== 3 || test.overlay() !== 'WEBGL CONTEXT FAILED') {
    throw new Error('repeated context-loss cap failed');
  }
  canvas.dispatchEvent(new Event('webglcontextrestored'));
  if (!test.context().terminal) throw new Error('terminal context unexpectedly reloaded');
  done({ok:true, left7, right7, leftFinger:leftFinger.world, rightFinger:rightFinger.world,
    caps, cameraRequests:observed});
} catch (error) {
  done({ok:false, error:String(error && error.stack || error)});
}

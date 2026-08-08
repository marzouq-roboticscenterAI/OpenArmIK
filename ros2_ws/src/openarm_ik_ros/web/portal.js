/* SPDX-License-Identifier: Apache-2.0 */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const csrf = document.querySelector('meta[name="portal-csrf"]').content;
  const targets = JSON.parse($('portal-targets').textContent);
  const demos = JSON.parse($('portal-demos').textContent);
  const sequences = JSON.parse($('portal-sequences').textContent);
  const demoById = Object.fromEntries(demos.map(entry => [entry.id, entry]));
  let sequenceRunning = false;
  const axes = ['x', 'y', 'z'];
  const sides = ['left', 'right'];
  const metresPerUnit = {m: 1.0, cm: 0.01, in: 0.0254};
  const unitsPerMetre = {m: 1.0, cm: 100.0, in: 1.0 / 0.0254};
  const unitDigits = {m: 6, cm: 4, in: 6};
  const unitNames = {m: 'metres (m)', cm: 'centimetres (cm)', in: 'inches (in)'};
  const decimalPattern = /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/;
  const targetsM = {left: [null, null, null], right: [null, null, null]};
  const fieldState = {};
  let unit = 'cm';
  let seeded = false;
  let measuredM = null;
  let readOnlyRealMode = false;
  let realMotionBlocked = false;
  const fieldId = (side, index) => (side === 'left' ? 'l' : 'r') + axes[index];
  for (const side of sides) for (let index = 0; index < 3; ++index) {
    fieldState[fieldId(side, index)] = {valid: false, touched: false};
  }
  const parseDecimal = text => decimalPattern.test(text) && Number.isFinite(Number(text)) ? Number(text) : null;
  const setError = message => {$('form-error').textContent = message;};
  const clearError = () => setError('');
  const allFieldsValid = () => sides.every(side => axes.every((axis, index) => fieldState[fieldId(side, index)].valid));
  function motionLimitScale() {
    const percent = Number($('motion-limit-scale').value);
    return Number.isFinite(percent) && percent >= 50 && percent <= 100 ? percent / 100 : null;
  }
  function updateMotionLimit() {
    const scale = motionLimitScale();
    if (scale === null) {setError('Movement limits must remain between 50% and 100%.'); return;}
    $('motion-limit-value').textContent = (scale * 100).toFixed(0) + '%';
  }
  function renderField(side, index) {
    const id = fieldId(side, index);
    const value = targetsM[side][index];
    if (value === null) return;
    $(id).value = (value * unitsPerMetre[unit]).toFixed(unitDigits[unit]);
    fieldState[id].valid = true;
  }
  const renderSide = side => {for (let index = 0; index < 3; ++index) renderField(side, index);};
  const renderAll = () => sides.forEach(renderSide);
  function updateUnitText() {
    $('unit-heading').textContent = unitNames[unit];
    for (const side of sides) for (let index = 0; index < 3; ++index) {
      const id = fieldId(side, index);
      $(id + '-label').textContent = axes[index].toUpperCase() + ' (' + unit + ')';
      $(id).placeholder = unit === 'cm' ? 'e.g. 45.0000' :
        (unit === 'in' ? 'e.g. 17.716535' : 'e.g. 0.450000');
    }
  }
  const syncUnitRadios = () => document.querySelectorAll('input[name="coordinate-unit"]').forEach(radio => {radio.checked = radio.value === unit;});
  function selectUnit(next) {
    if (!allFieldsValid()) {
      syncUnitRadios();
      setError('Correct every XYZ field before switching coordinate display units. Use a period (.) as the decimal separator.');
      return;
    }
    unit = next;
    updateUnitText();
    renderAll();
    clearError();
    $('form-notice').textContent = 'Targets are displayed in ' + unitNames[unit] + '. The underlying metre targets were preserved.';
  }
  function validateField(side, index) {
    const id = fieldId(side, index);
    const value = parseDecimal($(id).value);
    fieldState[id].touched = true;
    if (value === null) {
      fieldState[id].valid = false;
      setError('Enter a finite ASCII decimal for ' + axes[index].toUpperCase() + ' (' + unit + '). Blanks, whitespace, commas, hexadecimal, NaN, and infinity are not accepted.');
      return false;
    }
    targetsM[side][index] = value * metresPerUnit[unit];
    fieldState[id].valid = true;
    if (allFieldsValid()) clearError();
    return true;
  }
  function preset(side, id) {
    const entry = id === 'current' ? null : targets[side].find(target => target.id === id);
    const value = id === 'current' ? measuredM && measuredM[side] : entry && entry.point;
    if (!value) {setError('Fresh measured state is not available for that preset.'); return;}
    targetsM[side] = value.slice();
    for (let index = 0; index < 3; ++index) fieldState[fieldId(side, index)].touched = true;
    renderSide(side);
    clearError();
    $('form-notice').textContent = 'Fields filled in ' + unitNames[unit] + ' only; review values and press Move to submit.';
  }
  // Demo buttons fill both sides at once; a demo pose is a bimanual pair.
  function applyDemo(entry) {
    for (const side of sides) {
      targetsM[side] = entry[side].slice();
      for (let index = 0; index < 3; ++index) {
        fieldState[fieldId(side, index)].touched = true;
      }
      renderSide(side);
    }
    clearError();
    $('form-notice').textContent =
      'Filled both targets from "' + entry.label + '"; review values and press Move to submit.';
  }
  // Posts one dual move and resolves once the portal reports itself idle again.
  // Sending the next waypoint before then is refused with 409, because the
  // session holds its reservation until the goal is terminal.
  async function runWaypoint(entry, terminalRetreat = false) {
    const scale = motionLimitScale();
    if (scale === null) throw new Error('Movement limits must remain between 50% and 100%.');
    const body = entry.contact_target ? {
      side: 'left', unit: 'm', x: entry.contact_target[0],
      y: entry.contact_target[1], z: entry.contact_target[2],
      motion_limit_scale: scale,
    } : {
      unit: 'm',
      left_x: entry.left[0], left_y: entry.left[1], left_z: entry.left[2],
      right_x: entry.right[0], right_y: entry.right[1], right_z: entry.right[2],
      motion_limit_scale: scale,
    };
    // The guard rejects a submission whose measured state moved while it was
    // being evaluated and says so with "retry". That is a race detector doing
    // its job, not a bad request, so honour it instead of failing the sequence.
    let lastError = null;
    const backoffMs = [300, 600, 1000, 1500, 2000, 3000];
    for (let attempt = 0; attempt <= backoffMs.length; ++attempt) {
      try {
        await post(entry.contact_target ? '/api/v3/converge' :
          (terminalRetreat ? '/api/v3/retreat' : '/api/v3/move-both'), body);
        lastError = null;
        break;
      } catch (error) {
        lastError = error;
        if (!/retry|another portal goal is active/i.test(error.message)) throw error;
        if (attempt === backoffMs.length) break;
        await new Promise(resolve => window.setTimeout(resolve, backoffMs[attempt]));
      }
    }
    if (lastError) throw lastError;
    const deadline = Date.now() + 60000;
    let settled = 0;
    while (Date.now() < deadline) {
      await new Promise(resolve => window.setTimeout(resolve, 200));
      const response = await fetch('/api/state', {cache: 'no-store', credentials: 'same-origin'});
      const value = await response.json();
      settled = (value.state_fresh && !value.command_active) ? settled + 1 : 0;
      if (settled >= 3) {
        const completed = entry.contact_target ?
          /^Scoped contact result: converge_halted_on_proved_contact/.test(value.command) :
          /^Guarded route completed all \d+ leg\(s\) with \d+ measured-feedback replan\(s\): completed_measured_feedback/.test(value.command);
        if (!completed) {
          throw new Error('Waypoint stopped before completion: ' + value.command);
        }
        return;
      }
    }
    throw new Error('Waypoint did not finish within 60 s.');
  }
  async function setSceneBox(enabled) {
    try {
      await post('/api/scene-box', {enabled});
    } catch (_) { /* the prop is cosmetic; a failure must not stop the demo */ }
  }
  async function runSequence(sequence) {
    if (sequenceRunning) return;
    sequenceRunning = true;
    // The box prop belongs to pick-and-place only. Every other sequence sweeps
    // through where it sits and would grasp or shove it, so it is removed from
    // the scene for the duration of anything else.
    await setSceneBox(sequence.id === 'pick_place');
    setSequenceButtons(true);
    clearError();
    try {
      let steps = sequence.steps.slice();
      if (sequence.entry_steps) {
        const response = await fetch('/api/state', {cache: 'no-store', credentials: 'same-origin'});
        const latest = await response.json();
        if (!response.ok || !latest.state_fresh) {
          throw new Error('Fresh measured state is required before starting a contact demo.');
        }
        const open = demoById.clap_open;
        const squaredDistance = side => open[side].reduce((sum, value, axis) => {
          const delta = latest[side][axis] - value;
          return sum + delta * delta;
        }, 0);
        // A completed Heart/Clap leaves both claws at clap_open, so a repeat
        // starts immediately. Fresh startup and low/mid poses take the exact
        // branch-continuous entry that is covered by the native guard tests.
        if (squaredDistance('left') > 0.0025 || squaredDistance('right') > 0.0025) {
          steps = sequence.entry_steps.concat(steps);
        }
      }
      let previousWasContact = false;
      for (let index = 0; index < steps.length; ++index) {
        const entry = demoById[steps[index]];
        $('demo-progress').textContent =
          sequence.label + ': step ' + (index + 1) + ' of ' + steps.length +
          ' (' + entry.label + ')';
        applyDemo(entry);
        await runWaypoint(entry, previousWasContact && !entry.contact_target);
        previousWasContact = Boolean(entry.contact_target);
      }
      $('demo-progress').textContent = sequence.label + ': finished.';
    } catch (error) {
      $('demo-progress').textContent = sequence.label + ': stopped.';
      setError(error.message);
    } finally {
      // Marker topics are transient-local: an ADD remains in RViz until an
      // explicit DELETE arrives. Always remove the pick/place prop, including
      // after a stopped or failed sequence.
      if (sequence.id === 'pick_place') await setSceneBox(false);
      sequenceRunning = false;
      setSequenceButtons(false);
    }
  }
  function setSequenceButtons(disabled) {
    for (const sequence of sequences) {
      const button = $('demo-seq-' + sequence.id);
      if (button) button.disabled = disabled || realMotionBlocked;
    }
  }
  function renderDemoSequences() {
    const container = $('demo-sequences');
    if (!container) return;
    for (const sequence of sequences) {
      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'preset';
      button.id = 'demo-seq-' + sequence.id;
      button.textContent = sequence.label;
      button.addEventListener('click', () => runSequence(sequence));
      container.append(button);
    }
  }
  function renderDemoPresets() {
    const container = $('demo-presets');
    if (!container) return;
    for (const entry of demos) {
      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'preset';
      button.dataset.demo = entry.id;
      button.textContent = entry.label;
      button.addEventListener('click', () => applyDemo(entry));
      container.append(button);
    }
  }
  function renderPresets(side) {
    const container = $(side + '-presets');
    const entries = [{id: 'current', label: 'Current measured'}].concat(targets[side]);
    for (const entry of entries) {
      const button = document.createElement('button');
      button.type = 'button'; button.className = 'preset'; button.dataset.side = side; button.dataset.preset = entry.id;
      button.textContent = entry.label;
      button.addEventListener('click', () => preset(side, entry.id));
      container.append(button);
    }
  }
  async function state() {
    try {
      const response = await fetch('/api/state', {cache: 'no-store', credentials: 'same-origin'});
      const value = await response.json();
      if (!response.ok || value.coordinate_unit !== 'm') throw new Error('unexpected state response');
      $('status').textContent = value.command;
      const ok = value.state_fresh && !value.command_active && !readOnlyRealMode;
      $('left').disabled = !ok; $('right').disabled = !ok; $('both').disabled = !ok; $('age').textContent = value.summary;
      if (value.state_fresh) {
        measuredM = {left: value.left, right: value.right};
        if (!seeded) {
          for (const side of sides) for (let index = 0; index < 3; ++index) {
            const id = fieldId(side, index);
            if (!fieldState[id].touched) {targetsM[side][index] = measuredM[side][index]; renderField(side, index);}
          }
          seeded = true;
        }
      }
    } catch (_) {$('age').textContent = 'State unavailable'; $('left').disabled = true; $('right').disabled = true;}
  }
  async function post(path, body = {}) {
    const response = await fetch(path, {method: 'POST', credentials: 'same-origin', headers: {'Content-Type': 'application/json', 'X-CSRF-Token': csrf}, body: JSON.stringify(body)});
    const value = await response.json();
    if (!response.ok) throw new Error(value.error || 'request rejected');
    $('status').textContent = value.message;
    return value;
  }
  function move(side) {
    for (let index = 0; index < 3; ++index) {
      const id = fieldId(side, index);
      if (!fieldState[id].valid || targetsM[side][index] === null) {setError('Correct every XYZ field before submitting. Use a period (.) as the decimal separator.'); return;}
    }
    const values = targetsM[side].map(value => value * unitsPerMetre[unit]);
    const motion_limit_scale = motionLimitScale();
    if (motion_limit_scale === null) {setError('Movement limits must remain between 50% and 100%.'); return;}
    post('/api/v3/move', {side, unit, x: values[0], y: values[1], z: values[2], motion_limit_scale}).then(result => {
      $('form-notice').textContent = result.projected ?
        'The requested ' + side + ' target was impossible or unsafe. The sampled virtual guard queued only its farthest validated straight-line prefix (' + (result.achieved_fraction * 100).toFixed(2) + '%) at ' + (result.motion_limit_scale * 100).toFixed(0) + '% movement limits.' :
        'Submitted exact ' + side + ' target in ' + unitNames[unit] + ' at ' + (result.motion_limit_scale * 100).toFixed(0) + '% movement limits; the server normalized it once to metres.';
    }).catch(error => setError(error.message));
  }
  // Both arms in one atomic paired command. The server never shortens a dual
  // request, so there is no projected/achieved_fraction case to report here.
  function moveBoth() {
    for (const side of sides) {
      for (let index = 0; index < 3; ++index) {
        const id = fieldId(side, index);
        if (!fieldState[id].valid || targetsM[side][index] === null) {
          setError('Correct every XYZ field on both arms before submitting. Use a period (.) as the decimal separator.');
          return;
        }
      }
    }
    const left = targetsM.left.map(value => value * unitsPerMetre[unit]);
    const right = targetsM.right.map(value => value * unitsPerMetre[unit]);
    const motion_limit_scale = motionLimitScale();
    if (motion_limit_scale === null) {setError('Movement limits must remain between 50% and 100%.'); return;}
    post('/api/v3/move-both', {
      unit,
      left_x: left[0], left_y: left[1], left_z: left[2],
      right_x: right[0], right_y: right[1], right_z: right[2],
      motion_limit_scale,
    }).then(result => {
      $('form-notice').textContent =
        'Submitted both targets in ' + unitNames[unit] + ' as one paired command at ' +
        (result.motion_limit_scale * 100).toFixed(0) + '% movement limits; the arms move together.';
    }).catch(error => setError(error.message));
  }
  for (const side of sides) for (let index = 0; index < 3; ++index) $(fieldId(side, index)).addEventListener('input', () => validateField(side, index));
  document.querySelectorAll('input[name="coordinate-unit"]').forEach(radio => radio.addEventListener('change', () => {if (radio.checked) selectUnit(radio.value);}));
  $('motion-limit-scale').addEventListener('input', updateMotionLimit);
  $('left').addEventListener('click', () => move('left')); $('right').addEventListener('click', () => move('right'));
  $('both').addEventListener('click', moveBoth);
  $('stop').addEventListener('click', () => post('/api/stop').catch(error => {$('status').textContent = error.message;}));
  $('verify').addEventListener('click', () => post('/api/verify').catch(error => {$('status').textContent = error.message;}));
  // Real-arm mode. The portal serves /api/real/status in both modes, so one
  // request tells the page which stack it is attached to. In virtual mode this
  // returns {enabled:false} and nothing below runs, which is why the same page
  // works for run.sh and run-real.sh without a build-time switch.
  function renderRealPanel() {
    const panel = document.createElement('section');
    panel.id = 'real-panel';
    panel.className = 'controls';
    panel.innerHTML =
      '<h2>Physical arm</h2>' +
      '<p id="real-detail">Passive. Motors are disabled.</p>' +
      '<p><button id="real-connect" type="button">Connect and enable motors</button></p>' +
      '<p><button id="real-neutral" type="button" disabled>Return both arms to Neutral</button></p>' +
      '<p><button id="real-estop" type="button" class="stop">EMERGENCY STOP</button> ' +
      '<button id="real-estop-release" type="button" disabled>Release E-stop (motors stay disabled)</button></p>' +
      '<p id="real-estop-note" class="caption">The software E-stop has supremacy over motion: '
      + 'it cancels the active command, sends repeated disable frames to all 16 motors, and '
      + 'closes both CAN sockets. It is not a substitute for the hardwired stop.</p>' +
      '<p class="caption">Connect requires fresh encoder feedback from motor IDs 1..8 on both '
      + 'arms. Every target is seeded from the measured pose, gains ramp in gradually, and '
      + 'Disconnect disables every motor. Neutral uses live collision-aware route recovery.</p>' +
      '<p id="real-confidence"></p>' +
      '<p id="real-inventory"></p>' +
      '<p class="notice">The box in Pick/place is an RViz-only prop. The physical arms mimic ' +
      'the trajectory in empty space; the grippers hold their measured opening.</p>';
    document.querySelector('main').prepend(panel);
    return panel;
  }
  function describeBus(bus) {
    const name = bus.interface || bus.name || 'CAN';
    const count = bus.motor_count || (Array.isArray(bus.motors) ? bus.motors.length : bus.motors) || 0;
    if (!count) return name + ': silent (no motors answered)';
    const ids = Array.isArray(bus.motors) ?
      ' [' + bus.motors.map(m => '0x' + m.send_id.toString(16).padStart(2, '0')).join(' ') + ']' : '';
    return name + ': ' + count + ' motors as the ' + bus.side + ' arm' + ids;
  }
  function setRealOverlay(text) {
    const frame = document.querySelector('.viewer .frame');
    if (!frame) return;
    let overlay = document.getElementById('real-overlay');
    if (!text) {
      if (overlay) overlay.remove();
      return;
    }
    if (!overlay) {
      overlay = document.createElement('div');
      overlay.id = 'real-overlay';
      overlay.className = 'viewer-overlay';
      frame.appendChild(overlay);
    }
    overlay.textContent = text;
  }
  function applyRealStatus(observer) {
    if (!observer) return;
    // A motionless robot looks identical whether the stack is passive or the
    // arms are simply still, so say which it is on the view itself.
    setRealOverlay(observer.estop ? 'E-STOP LATCHED \u2014 motors disabled' : (!observer.connected ?
      'PASSIVE \u2014 press Connect to enable the arms' :
      (!observer.resolved ? 'CONNECTED, but the arms are not identified' : '')));
    $('real-detail').textContent = observer.detail || '';
    readOnlyRealMode = !observer.armed || Boolean(observer.busy);
    realMotionBlocked = readOnlyRealMode;
    setSequenceButtons(sequenceRunning);
    $('real-connect').disabled = Boolean(observer.estop);
    $('real-connect').textContent = observer.connected ?
      'Disconnect and disable motors' : 'Connect and enable motors';
    $('real-neutral').disabled = !observer.armed || Boolean(observer.busy);
    $('real-estop-release').disabled = !observer.estop;
    // The angle heuristic has been measured getting a real arm backwards, so
    // say so rather than presenting a guess as a determination.
    $('real-confidence').textContent = !observer.resolved ? '' :
      (observer.confidence === 'high' ?
        'Arm assignment confirmed.' :
        'Arm assignment is an UNVERIFIED GUESS. Move one arm by hand and watch which ' +
        'side moves on screen. If it is the wrong side, press Swap arms.');
    $('real-inventory').textContent = (observer.buses || []).map(describeBus).join('  |  ');
  }
  async function pollRealStatus() {
    try {
      const response = await fetch('/api/real/status', {credentials: 'same-origin'});
      applyRealStatus((await response.json()).observer);
    } catch (_) { /* transient; the next tick retries */ }
  }
  async function initRealMode() {
    let status;
    try {
      status = await (await fetch('/api/real/status', {credentials: 'same-origin'})).json();
    } catch (_) {return;}
    if (!status.enabled) return;
    readOnlyRealMode = true;
    // Real-arm commissioning is intentionally pinned to the slowest exposed
    // trajectory scale.  The physical backend independently enforces the
    // same cap, so this UI setting is truthful rather than the safety gate.
    $('motion-limit-scale').value = '50';
    $('motion-limit-scale').max = '50';
    updateMotionLimit();
    renderRealPanel();
    $('real-connect').addEventListener('click', async () => {
      $('real-connect').disabled = true;
      const disconnecting = status.observer && status.observer.connected;
      $('real-detail').textContent = disconnecting ?
        'Disabling all motors...' : 'Reading all 16 encoders before enable...';
      try {
        const latest = await (await fetch('/api/real/status', {credentials: 'same-origin'})).json();
        const path = latest.observer && latest.observer.connected ?
          '/api/real/disconnect' : '/api/real/connect';
        const result = await post(path);
        $('real-detail').textContent = result.message;
      } catch (error) {
        $('real-detail').textContent = error.message;
      }
      pollRealStatus();
    });
    $('real-neutral').addEventListener('click', async () => {
      try {
        const result = await post('/api/real/neutral');
        $('real-detail').textContent = result.message;
      } catch (error) {$('real-detail').textContent = error.message;}
      pollRealStatus();
    });
    applyRealStatus(status.observer);
    // The E-stop must work whether or not anything is connected, so it is wired
    // outside the status-driven enable/disable logic and never disabled.
    $('real-estop').addEventListener('click', async () => {
      $('real-estop-note').textContent = 'Stopping...';
      let outcome = 'controller confirmed disable';
      try {
        const result = await post('/api/estop');
        if (!result.physical_disabled) outcome = 'FAILED: ' + result.note;
      } catch (error) {outcome = 'FAILED: ' + error.message;}
      $('real-estop-note').textContent = 'E-STOP asserted; ' + outcome +
        '. Press the hardware stop if anything is still moving.';
      pollRealStatus();
    });
    $('real-estop-release').addEventListener('click', async () => {
      try {
        await post('/api/estop/release');
        $('real-estop-note').textContent =
          'E-stop released. Motors remain disabled; press Connect when the workspace is clear.';
      } catch (error) {$('real-estop-note').textContent = error.message;}
      pollRealStatus();
    });
    $('real-connect').focus();
    window.setInterval(pollRealStatus, 1000);
  }
  renderPresets('left'); renderPresets('right'); renderDemoPresets(); renderDemoSequences(); updateUnitText(); updateMotionLimit(); syncUnitRadios(); state(); window.setInterval(state, 250);
  // Mouse-driven RViz. The stream is one-way pixels, so orbiting works by
  // replaying pointer events back into the real RViz window server-side.
  function initRvizInput() {
    const image = $('rviz-stream');
    if (!image) return;
    let dragging = 0;
    let pending = null;
    let inFlight = false;

    // object-fit:contain letterboxes the frame inside the element, so the
    // element box is not the picture. Map through the drawn content rectangle
    // or every coordinate is offset and scaled wrongly.
    function normalise(event) {
      const box = image.getBoundingClientRect();
      const naturalW = image.naturalWidth || box.width;
      const naturalH = image.naturalHeight || box.height;
      const scale = Math.min(box.width / naturalW, box.height / naturalH);
      const drawnW = naturalW * scale;
      const drawnH = naturalH * scale;
      const originX = box.left + (box.width - drawnW) / 2;
      const originY = box.top + (box.height - drawnH) / 2;
      return {
        x: Math.min(1, Math.max(0, (event.clientX - originX) / drawnW)),
        y: Math.min(1, Math.max(0, (event.clientY - originY) / drawnH)),
      };
    }

    // One request in flight at a time, keeping only the newest move. A drag
    // generates events far faster than the round trip, and queueing them all
    // would make the view lag seconds behind the cursor.
    async function flush() {
      if (inFlight || !pending) return;
      const body = pending;
      pending = null;
      inFlight = true;
      try {
        const response = await fetch('/api/rviz/input', {
          method: 'POST', credentials: 'same-origin',
          headers: {'Content-Type': 'application/json', 'X-CSRF-Token': csrf},
          body: JSON.stringify(body),
        });
        if (!response.ok) {
          const value = await response.json().catch(() => ({}));
          if (value.error) $('status').textContent = value.error;
        }
      } catch (_) { /* dropped frame of input; the next event supersedes it */ }
      inFlight = false;
      if (pending) flush();
    }
    function send(body, coalesce) {
      // Presses and releases must never be dropped or reordered, or RViz is
      // left believing a button is still held.
      if (coalesce) {pending = body; flush(); return;}
      const previous = pending;
      pending = body;
      if (previous && previous.kind === 'move') { /* superseded, fine */ }
      flush();
    }

    image.addEventListener('pointerdown', event => {
      const point = normalise(event);
      dragging = event.button + 1;
      image.classList.add('dragging');
      image.setPointerCapture(event.pointerId);
      send({kind: 'press', x: point.x, y: point.y, button: dragging}, false);
      event.preventDefault();
    });
    image.addEventListener('pointermove', event => {
      if (!dragging) return;
      const point = normalise(event);
      send({kind: 'move', x: point.x, y: point.y, button: dragging}, true);
      event.preventDefault();
    });
    const endDrag = event => {
      if (!dragging) return;
      const point = normalise(event);
      send({kind: 'release', x: point.x, y: point.y, button: dragging}, false);
      dragging = 0;
      image.classList.remove('dragging');
      event.preventDefault();
    };
    image.addEventListener('pointerup', endDrag);
    image.addEventListener('pointercancel', endDrag);
    image.addEventListener('wheel', event => {
      const point = normalise(event);
      send({kind: 'wheel', x: point.x, y: point.y,
            notches: event.deltaY < 0 ? 1 : -1}, false);
      event.preventDefault();
    }, {passive: false});
    // RViz uses right-drag to zoom, so the browser menu has to stay out of it.
    image.addEventListener('contextmenu', event => event.preventDefault());
  }

  initRealMode();
  initRvizInput();
})();

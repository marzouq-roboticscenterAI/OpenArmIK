/* SPDX-License-Identifier: Apache-2.0 */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const csrf = document.querySelector('meta[name="portal-csrf"]').content;
  const targets = JSON.parse($('portal-targets').textContent);
  const demos = JSON.parse($('portal-demos').textContent);
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
      const ok = value.state_fresh && !value.command_active;
      $('left').disabled = !ok; $('right').disabled = !ok; $('age').textContent = value.summary;
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
  for (const side of sides) for (let index = 0; index < 3; ++index) $(fieldId(side, index)).addEventListener('input', () => validateField(side, index));
  document.querySelectorAll('input[name="coordinate-unit"]').forEach(radio => radio.addEventListener('change', () => {if (radio.checked) selectUnit(radio.value);}));
  $('motion-limit-scale').addEventListener('input', updateMotionLimit);
  $('left').addEventListener('click', () => move('left')); $('right').addEventListener('click', () => move('right'));
  $('stop').addEventListener('click', () => post('/api/stop').catch(error => {$('status').textContent = error.message;}));
  $('verify').addEventListener('click', () => post('/api/verify').catch(error => {$('status').textContent = error.message;}));
  renderPresets('left'); renderPresets('right'); renderDemoPresets(); updateUnitText(); updateMotionLimit(); syncUnitRadios(); state(); window.setInterval(state, 250);
})();

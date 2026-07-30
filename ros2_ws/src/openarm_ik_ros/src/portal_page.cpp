// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/portal_core.hpp"

#include <stdexcept>
#include <string>
#include <string_view>

namespace openarm_ik_ros::portal
{

std::string portal_page(std::string_view csrf)
{
  std::string html = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OpenArm virtual portal</title><style>
:root{color-scheme:dark;font-family:Inter,system-ui,sans-serif;background:#0b1018;color:#e9eef6}*{box-sizing:border-box}body{margin:0}main{display:grid;grid-template-columns:minmax(320px,420px) 1fr;min-height:100vh}.controls{padding:24px;background:#121a26;border-right:1px solid #2a3545;overflow:auto}.viewer{display:flex;flex-direction:column;padding:18px;min-width:0}.truth{background:#402713;border:1px solid #b36b27;border-radius:8px;padding:12px;margin:0 0 16px;line-height:1.4}.card{background:#192331;border:1px solid #304055;border-radius:10px;padding:14px;margin-bottom:12px}h1{font-size:1.4rem;margin:0 0 8px}h2{font-size:1rem;margin:0 0 10px}.xyz{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.presets{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}label{font-size:.75rem;color:#aeb9c8}.coordinate{width:100%;margin-top:4px;padding:9px;background:#0e1621;color:#fff;border:1px solid #41516a;border-radius:6px}.units{border:0;padding:0;margin:10px 0 0}.units legend{font-size:.8rem;margin-bottom:6px}.unit-options{display:flex;gap:18px}.unit-options label{display:flex;align-items:center;gap:6px;font-size:.82rem}.unit-options input{margin:0}button{width:100%;padding:10px;margin-top:10px;border:0;border-radius:7px;background:#2d77d0;color:white;font-weight:650;cursor:pointer}button.preset{background:#3f526c;font-size:.72rem;padding:8px}button.stop{background:#a13636}button.verify{background:#526275}button:disabled{opacity:.45;cursor:not-allowed}.status{font-family:ui-monospace,monospace;font-size:.82rem;white-space:pre-wrap;line-height:1.5}.error{color:#ffb3b3;font-size:.82rem;min-height:1.25rem;margin-top:8px}.notice{color:#b9c8da;font-size:.8rem;min-height:1.2rem;margin-top:5px}.frame{flex:1;display:flex;align-items:center;justify-content:center;background:#06090e;border:1px solid #2b3747;border-radius:10px;overflow:hidden;min-height:280px}.frame img{max-width:100%;max-height:calc(100vh - 80px);object-fit:contain}.caption{color:#9eabbc;font-size:.8rem;margin-top:8px}@media(max-width:900px){main{grid-template-columns:1fr}.controls{border-right:0}.viewer{min-height:55vh}.frame img{max-height:70vh}}
</style></head><body><main><section class="controls"><h1>OpenArm virtual portal</h1>
<div class="truth"><strong>Virtual simulation only.</strong><br>Controller collision checked: <strong>NO</strong>.<br>The portal uses sampled nominal capsules and a central keepout. This is not physical collision certification. A path is rejected unless that limited guard can prove its checks.</div>
<div class="card"><h2>Measured TCP / target — <span id="unit-heading">centimetres (cm)</span>, openarm_body_link0</h2><div class="caption">+X forward, +Y left, +Z up. Presets are virtual-model, sampled-nominal-guard test values only—not physically safe coordinates. Preset buttons only fill fields; they never submit motion.</div><fieldset class="units"><legend>Coordinate display units</legend><div class="unit-options"><label><input type="radio" name="coordinate-unit" value="cm" checked>Centimetres (cm)</label><label><input type="radio" name="coordinate-unit" value="in">Inches (in)</label></div></fieldset><div class="caption">ROS and RViz geometry remains metric. The captured stock RViz view has no portal-switchable coordinate grid.</div><div id="age" class="caption">Waiting for encoder-derived joint state…</div></div>
<div class="card"><h2>Left target (orientation unconstrained)</h2><div class="xyz"><label><span id="lx-label">X (cm)</span><input class="coordinate" id="lx" type="text" inputmode="decimal" required aria-labelledby="lx-label"></label><label><span id="ly-label">Y (cm)</span><input class="coordinate" id="ly" type="text" inputmode="decimal" required aria-labelledby="ly-label"></label><label><span id="lz-label">Z (cm)</span><input class="coordinate" id="lz" type="text" inputmode="decimal" required aria-labelledby="lz-label"></label></div><div class="presets"><button class="preset" data-side="left" data-preset="current">Current measured</button><button class="preset" data-side="left" data-preset="small">Small forward/up</button><button class="preset" data-side="left" data-preset="medium">Medium forward/up</button></div><button id="left" disabled>Move Left (Right target = freshest measured TCP)</button></div>
<div class="card"><h2>Right target (orientation unconstrained)</h2><div class="xyz"><label><span id="rx-label">X (cm)</span><input class="coordinate" id="rx" type="text" inputmode="decimal" required aria-labelledby="rx-label"></label><label><span id="ry-label">Y (cm)</span><input class="coordinate" id="ry" type="text" inputmode="decimal" required aria-labelledby="ry-label"></label><label><span id="rz-label">Z (cm)</span><input class="coordinate" id="rz" type="text" inputmode="decimal" required aria-labelledby="rz-label"></label></div><div class="presets"><button class="preset" data-side="right" data-preset="current">Current measured</button><button class="preset" data-side="right" data-preset="small">Small forward/up</button><button class="preset" data-side="right" data-preset="medium">Medium forward/up</button></div><button id="right" disabled>Move Right (Left target = freshest measured TCP)</button></div>
<div id="form-error" class="error" role="alert"></div><div id="form-notice" class="notice" aria-live="polite"></div>
<div class="card"><button class="verify" id="verify">Auto Calibrate — simulation verification only</button><div class="caption">Nonmoving model/state verification; it performs no physical calibration.</div><button class="stop" id="stop">Request software stop (not a hardwired E-stop)</button><div class="caption">Cancels the active portal goal. It is not safety-rated and cannot replace a hardwired E-stop.</div></div>
<div class="card"><h2>Measured command progress/result</h2><div id="status" class="status">No portal command.</div></div></section>
<section class="viewer"><h2>Actual launcher-owned stock RViz pixels</h2><div class="frame"><img id="rviz" alt="RViz capture unavailable"></div><div class="caption">XComposite snapshot from the exact launcher PID. Image freshness is never used as control feedback.</div></section></main>
<script>
const csrf='__CSRF__';
const samplesM={left:{small:[__LSX__,__LSY__,__LSZ__],medium:[__LMX__,__LMY__,__LMZ__]},right:{small:[__RSX__,__RSY__,__RSZ__],medium:[__RMX__,__RMY__,__RMZ__]}};
const axes=['x','y','z'],sides=['left','right'];
const metresPerUnit={cm:0.01,in:0.0254};
const unitsPerMetre={cm:100.0,in:1.0/0.0254};
const unitDigits={cm:4,in:6};
const unitNames={cm:'centimetres (cm)',in:'inches (in)'};
const decimalPattern=/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/;
const targetsM={left:[null,null,null],right:[null,null,null]};
const fieldState={};
let unit='cm',seeded=false,measuredM=null;
const $=id=>document.getElementById(id);
const fieldId=(side,index)=>(side==='left'?'l':'r')+axes[index];
for(const side of sides)for(let index=0;index<3;++index)fieldState[fieldId(side,index)]={valid:false,touched:false,edited:false};
function parseDecimal(text){if(!decimalPattern.test(text))return null;const value=Number(text);return Number.isFinite(value)?value:null}
function setError(message){$('form-error').textContent=message}
function clearError(){setError('')}
function allFieldsValid(){return sides.every(side=>axes.every((axis,index)=>fieldState[fieldId(side,index)].valid))}
function renderField(side,index){const id=fieldId(side,index),value=targetsM[side][index];if(value===null)return;$(id).value=(value*unitsPerMetre[unit]).toFixed(unitDigits[unit]);fieldState[id].valid=true;fieldState[id].edited=false}
function renderSide(side){for(let index=0;index<3;++index)renderField(side,index)}
function renderAll(){for(const side of sides)renderSide(side)}
function updateUnitText(){
  $('unit-heading').textContent=unitNames[unit];
  for(const side of sides)for(let index=0;index<3;++index){const id=fieldId(side,index);$(id+'-label').textContent=axes[index].toUpperCase()+' ('+unit+')';$(id).placeholder=unit==='cm'?'e.g. 2.0000':'e.g. 0.787402'}
}
function syncUnitRadios(){for(const radio of document.querySelectorAll('input[name="coordinate-unit"]'))radio.checked=radio.value===unit}
function selectUnit(next){if(!allFieldsValid()){syncUnitRadios();setError('Correct every XYZ field before switching coordinate display units. Use a period (.) as the decimal separator.');return}unit=next;updateUnitText();renderAll();clearError();$('form-notice').textContent='Targets are displayed in '+unitNames[unit]+'. The underlying metre targets were preserved.'}
function validateField(side,index){
  const id=fieldId(side,index),state=fieldState[id],value=parseDecimal($(id).value);state.touched=true;state.edited=true;
  if(value===null){state.valid=false;setError('Enter a finite ASCII decimal for '+axes[index].toUpperCase()+' ('+unit+'). Blanks, whitespace, commas, hexadecimal, NaN, and infinity are not accepted.');return false}
  targetsM[side][index]=value*metresPerUnit[unit];state.valid=true;if(allFieldsValid())clearError();return true;
}
function preset(side,name){const value=name==='current'?(measuredM&&measuredM[side]):samplesM[side][name];if(!value){setError('Fresh measured state is not available for that preset.');return}targetsM[side]=value.slice();for(let index=0;index<3;++index)fieldState[fieldId(side,index)].touched=true;renderSide(side);clearError();$('form-notice').textContent='Fields filled in '+unitNames[unit]+' only; review values and press Move to submit.'}
async function state(){
  try{const response=await fetch('/api/state',{cache:'no-store'}),value=await response.json();if(value.coordinate_unit!=='m')throw new Error('unexpected state coordinate unit');$('status').textContent=value.command;const ok=value.state_fresh&&!value.command_active;$('left').disabled=!ok;$('right').disabled=!ok;$('age').textContent=value.summary;
    if(value.state_fresh){measuredM={left:value.left,right:value.right};if(!seeded){for(const side of sides)for(let index=0;index<3;++index){const id=fieldId(side,index);if(!fieldState[id].touched){targetsM[side][index]=measuredM[side][index];renderField(side,index)}}seeded=true}}
  }catch(error){$('age').textContent='State unavailable';$('left').disabled=true;$('right').disabled=true}
}
async function post(path,body={}){const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf},body:JSON.stringify(body)});const value=await response.json();if(!response.ok)throw new Error(value.error||'request rejected');$('status').textContent=value.message}
function move(side){for(let index=0;index<3;++index){const id=fieldId(side,index);if(!fieldState[id].valid||targetsM[side][index]===null){setError('Correct every XYZ field before submitting. Use a period (.) as the decimal separator.');return}}const target=targetsM[side],values=target.map(value=>value*unitsPerMetre[unit]);post('/api/v2/move',{side,unit,x:values[0],y:values[1],z:values[2]}).then(()=>{$('form-notice').textContent='Submitted '+side+' target in '+unitNames[unit]+' from its preserved canonical metre values; the server normalized it once to metres.'}).catch(error=>setError(error.message))}
for(const side of sides)for(let index=0;index<3;++index)$(fieldId(side,index)).addEventListener('input',()=>validateField(side,index));
for(const button of document.querySelectorAll('button.preset'))button.addEventListener('click',()=>preset(button.dataset.side,button.dataset.preset));
for(const radio of document.querySelectorAll('input[name="coordinate-unit"]'))radio.addEventListener('change',()=>{if(radio.checked)selectUnit(radio.value)});
$('left').addEventListener('click',()=>move('left'));$('right').addEventListener('click',()=>move('right'));
$('stop').addEventListener('click',()=>post('/api/stop').catch(error=>$('status').textContent=error.message));
$('verify').addEventListener('click',()=>post('/api/verify').catch(error=>$('status').textContent=error.message));
updateUnitText();syncUnitRadios();setInterval(state,250);state();const image=$('rviz');setInterval(()=>{image.src='/api/rviz.jpg?t='+Date.now()},350);image.src='/api/rviz.jpg';
</script></body></html>)HTML";
  const NominalTestSamples left = nominal_test_samples(MoveRequest::Side::left);
  const NominalTestSamples right = nominal_test_samples(MoveRequest::Side::right);
  const auto replace = [&html](std::string_view key, std::string_view value) {
      const std::size_t position = html.find(key);
      if (position == std::string::npos) {
        throw std::logic_error("portal page placeholder missing");
      }
      html.replace(position, key.size(), value);
    };
  replace("__CSRF__", csrf);
  replace("__LSX__", json_number(left.small_forward_up[0]));
  replace("__LSY__", json_number(left.small_forward_up[1]));
  replace("__LSZ__", json_number(left.small_forward_up[2]));
  replace("__LMX__", json_number(left.medium_forward_up[0]));
  replace("__LMY__", json_number(left.medium_forward_up[1]));
  replace("__LMZ__", json_number(left.medium_forward_up[2]));
  replace("__RSX__", json_number(right.small_forward_up[0]));
  replace("__RSY__", json_number(right.small_forward_up[1]));
  replace("__RSZ__", json_number(right.small_forward_up[2]));
  replace("__RMX__", json_number(right.medium_forward_up[0]));
  replace("__RMY__", json_number(right.medium_forward_up[1]));
  replace("__RMZ__", json_number(right.medium_forward_up[2]));
  return html;
}

}  // namespace openarm_ik_ros::portal

// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/portal_core.hpp"

#include <sstream>
#include <string>
#include <string_view>

namespace openarm_ik_ros::portal
{
namespace
{
std::string targets_json()
{
  std::ostringstream output;
  output << "{\"left\":[";
  for (std::size_t side = 0; side < 2; ++side) {
    if (side != 0) {output << "],\"right\":[";}
    const NominalTargetTable & targets = nominal_targets(
      side == 0 ? MoveRequest::Side::left : MoveRequest::Side::right);
    for (std::size_t index = 0; index < targets.size(); ++index) {
      if (index != 0) {output << ',';}
      const NominalTarget & target = targets[index];
      output << "{\"id\":\"" << json_escape(target.id) << "\",\"label\":\"" <<
        json_escape(target.label) << "\",\"point\":[" << json_number(target.point[0]) << ',' <<
        json_number(target.point[1]) << ',' << json_number(target.point[2]) << "]}";
    }
  }
  output << "]}";
  return output.str();
}
}  // namespace

std::string portal_page(std::string_view csrf)
{
  return std::string(R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="portal-csrf" content=")HTML") + std::string(csrf) + R"HTML(">
<title>OpenArm virtual portal</title><link rel="stylesheet" href="/web/portal.css">
<script defer src="/web/portal.js"></script><script defer src="/web/viewer.js"></script></head><body>
<main><section class="controls"><h1>OpenArm virtual portal</h1>
<div class="truth"><strong>Virtual simulation only.</strong><br>Controller collision checked: <strong>NO</strong>.<br>The portal uses sampled nominal capsules and a central keepout. This is not physical collision certification. Impossible or unsafe requests are shortened to the farthest validated straight-line prefix; the first sampled keepout stops the search.</div>
<div class="card"><h2>Measured TCP / target — <span id="unit-heading">centimetres (cm)</span>, openarm_body_link0</h2><div class="caption">+X forward, +Y left, +Z up. Presets span a wide, virtual-model-validated workspace but are not physically certified coordinates. Preset buttons only fill fields; they never submit motion.</div><fieldset class="units"><legend>Coordinate display/input units</legend><div class="unit-options"><label><input type="radio" name="coordinate-unit" value="cm" checked>Centimetres (cm)</label><label><input type="radio" name="coordinate-unit" value="in">Inches (in)</label><label><input type="radio" name="coordinate-unit" value="m">Metres (m)</label></div></fieldset><div class="caption">All control coordinates remain IEEE-754 binary64. ROS and visualization geometry remain metric. The proxy has no portal-switchable coordinate grid.</div><div id="age" class="caption">Waiting for encoder-derived joint state…</div></div>
<div class="card"><h2>Movement limits — <span id="motion-limit-value">80%</span></h2><label class="motion-limit" for="motion-limit-scale">Configured velocity, acceleration, and jerk limit scale</label><input id="motion-limit-scale" type="range" min="50" max="100" step="5" value="80"><div class="caption">50% is the previous portal behavior; 100% is the virtual model's configured maximum. The smooth seventh-order trajectory remains bounded. This percentage scales three limits equally, so travel time is not linear.</div></div>
<div class="card"><h2>Virtual guard test inputs (cm)</h2><div class="caption"><strong>Near-full audited reach:</strong> use the High far preset: Left [28, 67, 52], Right [28, -67, 52].<br><strong>Impossible/reach projection:</strong> Left [5000, 5000, 5000], Right [5000, -5000, 5000]. From neutral, each makes a large best-effort move instead of commanding 50 m.<br><strong>Pole mitigation:</strong> Left [40, 5, 40], Right [40, -5, 40]. The sampled guard stops at its 2.5 cm nominal gate.<br><strong>Inter-arm mitigation:</strong> move both arms to their own Near-max forward presets, then request Left [48, -17, 35]. The left arm stops before the right-arm capsule gate.<br>These are virtual regression inputs, not physically certified poses.</div></div>
<div class="card"><h2>Left target (orientation unconstrained)</h2><div class="xyz"><label><span id="lx-label">X (cm)</span><input class="coordinate" id="lx" type="text" inputmode="decimal" required aria-labelledby="lx-label"></label><label><span id="ly-label">Y (cm)</span><input class="coordinate" id="ly" type="text" inputmode="decimal" required aria-labelledby="ly-label"></label><label><span id="lz-label">Z (cm)</span><input class="coordinate" id="lz" type="text" inputmode="decimal" required aria-labelledby="lz-label"></label></div><div id="left-presets" class="presets"></div><button id="left" disabled>Move Left (Right target = freshest measured TCP)</button></div>
<div class="card"><h2>Right target (orientation unconstrained)</h2><div class="xyz"><label><span id="rx-label">X (cm)</span><input class="coordinate" id="rx" type="text" inputmode="decimal" required aria-labelledby="rx-label"></label><label><span id="ry-label">Y (cm)</span><input class="coordinate" id="ry" type="text" inputmode="decimal" required aria-labelledby="ry-label"></label><label><span id="rz-label">Z (cm)</span><input class="coordinate" id="rz" type="text" inputmode="decimal" required aria-labelledby="rz-label"></label></div><div id="right-presets" class="presets"></div><button id="right" disabled>Move Right (Left target = freshest measured TCP)</button></div>
<div id="form-error" class="error" role="alert"></div><div id="form-notice" class="notice" aria-live="polite"></div>
<div class="card"><button class="verify" id="verify">Auto Calibrate — simulation verification only</button><div class="caption">Nonmoving model/state verification; it performs no physical calibration.</div><button class="stop" id="stop">Request software stop (not a hardwired E-stop)</button><div class="caption">Cancels the active portal goal. It is not safety-rated and cannot replace a hardwired E-stop.</div></div>
<div class="card"><h2>Measured command progress/result</h2><div id="status" class="status">No portal command.</div></div></section>
<section class="viewer"><h2>OpenArm measured-pose viewer</h2><div class="frame"><canvas id="viewer-canvas" aria-label="OpenArm measured-pose visual proxy"></canvas><div id="viewer-overlay" class="viewer-overlay" role="status">LOADING VIEWER</div></div><div class="viewer-tools"><button id="reset-view" type="button">Reset view</button><button id="viewer-neutral-palette" type="button" aria-pressed="false" aria-describedby="viewer-style-note">RViz-like neutral colors</button><span id="viewer-metrics">Viewer loading</span></div><div id="viewer-style-note" class="caption">The color button only changes this local WebGL proxy and never sends a robot command. Blue/light-blue is the default; neutral is RViz-like, not stock RViz rendering.</div><div class="caption"><strong>visual proxy — not collision checking</strong>. Collision STL geometry is shown only to visualize the latest measured pose. Viewer freshness is never used as control feedback.</div></section></main>
<script id="portal-targets" type="application/json">)HTML" + targets_json() + R"HTML(</script></body></html>)HTML";
}

}  // namespace openarm_ik_ros::portal

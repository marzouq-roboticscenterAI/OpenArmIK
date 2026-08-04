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

// The same waypoints the CLI demos use, all measured against the real-time
// keepout monitor. Metres, openarm_body_link0.
std::string demo_targets_json()
{
  return R"JSON([
{"id":"clap_open","label":"Clap: open","left":[0.30,0.26,0.35],"right":[0.30,-0.26,0.35]},
{"id":"clap_closed","label":"Clap: closed","left":[0.30,0.12,0.35],"right":[0.30,-0.12,0.35]},
{"id":"cross_open","label":"Cross: open","left":[0.30,0.26,0.45],"right":[0.30,-0.26,0.45]},
{"id":"cross_split","label":"Cross: stack heights","left":[0.30,0.26,0.62],"right":[0.30,-0.26,0.22]},
{"id":"cross_over","label":"Cross: claws swapped","left":[0.30,-0.04,0.62],"right":[0.30,0.04,0.22]},
{"id":"mirror","label":"Mirrored pair","left":[0.30,0.22,0.40],"right":[0.30,-0.22,0.40]},
{"id":"forward_mid","label":"Forward mid","left":[0.30,0.22,0.30],"right":[0.30,-0.22,0.30]},
{"id":"neutral_low","label":"Near low","left":[0.15,0.22,0.15],"right":[0.15,-0.22,0.15]},
{"id":"box_approach","label":"Box: approach","left":[0.34,0.24,0.30],"right":[0.34,-0.24,0.30]},
{"id":"box_grasp","label":"Box: close on it","left":[0.34,0.11,0.30],"right":[0.34,-0.11,0.30]},
{"id":"box_lift","label":"Box: lift","left":[0.34,0.11,0.48],"right":[0.34,-0.11,0.48]},
{"id":"box_carry","label":"Box: carry across","left":[0.26,0.11,0.50],"right":[0.26,-0.11,0.50]},
{"id":"box_place","label":"Box: lower to place","left":[0.26,0.11,0.32],"right":[0.26,-0.11,0.32]},
{"id":"box_release","label":"Box: release","left":[0.26,0.24,0.32],"right":[0.26,-0.24,0.32]}
])JSON";
}

std::string portal_page(std::string_view csrf)
{
  return std::string(R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="portal-csrf" content=")HTML") + std::string(csrf) + R"HTML(">
<title>OpenArm virtual portal</title><link rel="stylesheet" href="/web/portal.css">
<script defer src="/web/portal.js"></script></head><body>
<main><section class="controls"><h1>OpenArm virtual portal</h1>
<div class="truth"><strong>Virtual simulation only.</strong><br>Controller collision checked: <strong>NO</strong>.<br>The portal uses sampled nominal capsules and a central keepout. This is not physical collision certification. Impossible or unsafe requests are shortened to the farthest validated straight-line prefix; the first sampled keepout stops the search.</div>
<div class="card"><h2>Measured TCP / target — <span id="unit-heading">centimetres (cm)</span>, openarm_body_link0</h2><div class="caption">+X forward, +Y left, +Z up. Presets span a wide, virtual-model-validated workspace but are not physically certified coordinates. Preset buttons only fill fields; they never submit motion.</div><fieldset class="units"><legend>Coordinate display/input units</legend><div class="unit-options"><label><input type="radio" name="coordinate-unit" value="cm" checked>Centimetres (cm)</label><label><input type="radio" name="coordinate-unit" value="in">Inches (in)</label><label><input type="radio" name="coordinate-unit" value="m">Metres (m)</label></div></fieldset><div class="caption">All control coordinates remain IEEE-754 binary64. ROS and visualization geometry remain metric. The proxy has no portal-switchable coordinate grid.</div><div id="age" class="caption">Waiting for encoder-derived joint state…</div></div>
<div class="card"><h2>Movement limits — <span id="motion-limit-value">80%</span></h2><label class="motion-limit" for="motion-limit-scale">Configured velocity, acceleration, and jerk limit scale</label><input id="motion-limit-scale" type="range" min="50" max="100" step="5" value="80"><div class="caption">50% is the previous portal behavior; 100% is the virtual model's configured maximum. The smooth seventh-order trajectory remains bounded. This percentage scales three limits equally, so travel time is not linear.</div></div>
<div class="card"><h2>Virtual guard test inputs (cm)</h2><div class="caption"><strong>Near-full audited reach:</strong> use the High far preset: Left [28, 67, 52], Right [28, -67, 52].<br><strong>Impossible/reach projection:</strong> Left [5000, 5000, 5000], Right [5000, -5000, 5000]. From neutral, each makes a large best-effort move instead of commanding 50 m.<br><strong>Pole mitigation:</strong> Left [40, 5, 40], Right [40, -5, 40]. The sampled guard stops at its 2.5 cm nominal gate.<br><strong>Inter-arm mitigation:</strong> move both arms to their own Near-max forward presets, then request Left [48, -17, 35]. The left arm stops before the right-arm capsule gate.<br>These are virtual regression inputs, not physically certified poses.</div></div>
<div class="card"><h2>Demo poses</h2><div class="caption">Each button fills <strong>both</strong> the left and right target fields with one waypoint of a demo. Buttons only fill fields; they never submit motion. Every waypoint was measured against the real-time keepout monitor. Run a full sequence with <code>openarm_control_cli clap</code>, <code>cross</code>, or <code>pick-place</code>. The box is a visualization aid: it is not part of the keepout model.</div><div id="demo-presets" class="presets"></div></div>
<div class="card"><h2>Left target (orientation unconstrained)</h2><div class="xyz"><label><span id="lx-label">X (cm)</span><input class="coordinate" id="lx" type="text" inputmode="decimal" required aria-labelledby="lx-label"></label><label><span id="ly-label">Y (cm)</span><input class="coordinate" id="ly" type="text" inputmode="decimal" required aria-labelledby="ly-label"></label><label><span id="lz-label">Z (cm)</span><input class="coordinate" id="lz" type="text" inputmode="decimal" required aria-labelledby="lz-label"></label></div><div id="left-presets" class="presets"></div><button id="left" disabled>Move Left (Right target = freshest measured TCP)</button></div>
<div class="card"><h2>Right target (orientation unconstrained)</h2><div class="xyz"><label><span id="rx-label">X (cm)</span><input class="coordinate" id="rx" type="text" inputmode="decimal" required aria-labelledby="rx-label"></label><label><span id="ry-label">Y (cm)</span><input class="coordinate" id="ry" type="text" inputmode="decimal" required aria-labelledby="ry-label"></label><label><span id="rz-label">Z (cm)</span><input class="coordinate" id="rz" type="text" inputmode="decimal" required aria-labelledby="rz-label"></label></div><div id="right-presets" class="presets"></div><button id="right" disabled>Move Right (Left target = freshest measured TCP)</button></div>
<div id="form-error" class="error" role="alert"></div><div id="form-notice" class="notice" aria-live="polite"></div>
<div class="card"><button class="verify" id="verify">Auto Calibrate — simulation verification only</button><div class="caption">Nonmoving model/state verification; it performs no physical calibration.</div><button class="stop" id="stop">Request software stop (not a hardwired E-stop)</button><div class="caption">Cancels the active portal goal. It is not safety-rated and cannot replace a hardwired E-stop.</div></div>
<div class="card"><h2>Measured command progress/result</h2><div id="status" class="status">No portal command.</div></div></section>
<section class="viewer"><h2>RViz</h2><div class="frame"><img id="rviz-stream" src="/api/rviz/stream" alt="Live RViz 3D view"></div><div class="caption">These are real <strong>RViz</strong> pixels, captured from the running rviz2 process and streamed as MJPEG. The view is cropped to the 3D render area, so the Qt menus and toolbars are not shown. It is display-only: the stream sends no input to RViz.</div><div class="caption">RViz renders the measured pose of the pinned model. It is not collision checking, and its freshness is never used as control feedback.</div></section></main>
<script id="portal-targets" type="application/json">)HTML" + targets_json() + R"HTML(</script>
<script id="portal-demos" type="application/json">)HTML" + demo_targets_json() + R"HTML(</script></body></html>)HTML";
}

}  // namespace openarm_ik_ros::portal

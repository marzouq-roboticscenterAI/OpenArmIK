#!/usr/bin/env bash
# End-to-end functional sweep of every surface.
#
# Run against an already-started stack:
#   bash run.sh --no-build --no-browser &   # wait ~30 s for it to come up
#   scripts/functional_sweep.sh
#
# Covers the portal routes, the live RViz MJPEG stream, every CLI motion and
# demo, the graspable box, the rejection paths, and the emergency stop. It is
# deliberately ordered so converge runs before the retreat-dependent checks:
# converge ends with the claws inside the keepout intervention floor, and the
# demos after it only pass because the monitor permits a retreat from there.
cd /home/signalprocessing-dev/OpenArmIK
source /opt/ros/lyrical/setup.bash >/dev/null 2>&1
source ros2_ws/install/setup.bash >/dev/null 2>&1
pass=0; fail=0
check() { if [[ "$2" == "$3" ]]; then echo "  PASS  $1"; pass=$((pass+1));
          else echo "  FAIL  $1  (got '$2' want '$3')"; fail=$((fail+1)); fi }
cli() { timeout 240 ros2 run openarm_ik_ros openarm_control_cli "$@" 2>&1; }

CSRF=$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -oP '(?<=name="portal-csrf" content=")[0-9a-f]{64}')
H=(-H "Content-Type: application/json" -H "Origin: http://127.0.0.1:8080"
   -H "Sec-Fetch-Site: same-origin" -H "X-CSRF-Token: $CSRF")

echo "== portal routes =="
check "GET /api/health"   "$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 http://127.0.0.1:8080/api/health)" "200"
check "GET /"             "$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 http://127.0.0.1:8080/)" "200"
check "GET /api/state"    "$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 http://127.0.0.1:8080/api/state)" "200"
check "page has demo buttons" "$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -c 'id="demo-presets"')" "1"
check "page embeds stream"    "$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -c '/api/rviz/stream')" "1"
check "page has box preset"   "$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -c 'box_grasp')" "1"
check "page has cross preset" "$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -c 'cross_over')" "1"

echo "== MJPEG stream =="
python3 - <<'PY'
import socket,re,sys
try:
    s=socket.create_connection(("127.0.0.1",8080),timeout=10)
    s.sendall(b"GET /api/rviz/stream HTTP/1.1\r\nHost: 127.0.0.1:8080\r\nConnection: close\r\n\r\n")
    buf=b""
    while b"\r\n\r\n" not in buf: buf+=s.recv(4096)
    hdr,rest=buf.split(b"\r\n\r\n",1)
    ok = b"multipart/x-mixed-replace" in hdr
    while b"\r\n\r\n" not in rest: rest+=s.recv(4096)
    part,body=rest.split(b"\r\n\r\n",1)
    n=int(re.search(rb"Content-Length: (\d+)",part).group(1))
    while len(body)<n: body+=s.recv(65536)
    jpeg = body[:2]==b"\xff\xd8" and n>5000
    s.close()
    print("  PASS  multipart header" if ok else "  FAIL  multipart header")
    print("  PASS  jpeg frame %d bytes"%n if jpeg else "  FAIL  jpeg frame %d bytes"%n)
except Exception as e:
    print("  FAIL  stream:",e)
PY

echo "== CLI motion =="
check "status"          "$(cli status >/dev/null 2>&1; echo $?)" "0"
check "move-joint"      "$(cli move-joint openarm_left_joint4 0.3 >/dev/null 2>&1; echo $?)" "0"
check "move-paired-tcp" "$(cli move-paired-tcp openarm_body_link0 0.30 0.24 0.40 0.30 -0.24 0.40 >/dev/null 2>&1; echo $?)" "0"
check "mirror"          "$(cli mirror left 0.30 0.22 0.38 >/dev/null 2>&1; echo $?)" "0"
check "centroid"        "$(cli centroid 0.32 0.00 0.42 >/dev/null 2>&1; echo $?)" "0"
check "converge"        "$(cli converge 0.34 0.00 0.40 0.08 >/dev/null 2>&1; echo $?)" "0"
check "converge again"  "$(cli converge 0.34 0.00 0.42 0.06 >/dev/null 2>&1; echo $?)" "0"
check "clap"            "$(cli clap 1 >/dev/null 2>&1; echo $?)" "0"
check "cross"           "$(cli cross 1 >/dev/null 2>&1; echo $?)" "0"
check "home"            "$(cli home >/dev/null 2>&1; echo $?)" "0"
check "pick-place"      "$(cli pick-place >/dev/null 2>&1; echo $?)" "0"

echo "== box moved =="
BOX=$(timeout 10 ros2 topic echo /openarm_ik/scene_box --once 2>/dev/null | grep -A1 "position:" | grep "x:" | awk '{print $2}')
check "box carried to x~0.26" "$(python3 -c "print('yes' if abs(float('$BOX')-0.26)<0.03 else 'no($BOX)')")" "yes"

echo "== rejections =="
check "bad joint rejected"  "$(cli move-joint nope 0.1 >/dev/null 2>&1; echo $?)" "4"
check "bad frame rejected"  "$(cli move-paired-tcp nope 0.2 0.3 0.85 0.2 -0.3 0.85 >/dev/null 2>&1; echo $?)" "7"

echo "== E-stop =="
check "engage"  "$(curl -s --max-time 5 -X POST "${H[@]}" -d '{}' http://127.0.0.1:8080/api/estop | grep -c '"estop":"engaged"')" "1"
check "motion refused while latched" "$(curl -s --max-time 8 -X POST "${H[@]}" -d '{"side":"left","unit":"m","x":0.30,"y":0.22,"z":0.30,"motion_limit_scale":0.8}' http://127.0.0.1:8080/api/v3/move | grep -c 'emergency stop is engaged')" "1"
check "release" "$(curl -s --max-time 5 -X POST "${H[@]}" -d '{}' http://127.0.0.1:8080/api/estop/release | grep -c '"estop":"released"')" "1"
cli home >/dev/null 2>&1
check "motion works after release" "$(curl -s -o /dev/null -w '%{http_code}' --max-time 8 -X POST "${H[@]}" -d '{"side":"left","unit":"m","x":0.28,"y":0.20,"z":0.36,"motion_limit_scale":0.8}' http://127.0.0.1:8080/api/v3/move)" "202"

echo
echo "SWEEP: $pass passed, $fail failed"
exit $((fail>0))

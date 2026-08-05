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
# Motion commands must not be posted back to back: the session holds a
# reservation until the previous goal is terminal, and a racing goal is
# rejected with exit 4, which reads like a broken command rather than a
# sequencing mistake in this script.
climove() { wait_idle; cli "$@"; }
# The portal refuses a new goal while one is active, so wait for it to go idle
# rather than racing it. Without this the next POST returns 409 and the failure
# looks like a routing bug instead of a sequencing one.
# Requires fresh state and no active command, sustained across three polls.
# A single poll is not enough: right after a goal is posted command_active has
# not flipped true yet, and right after one ends the state is briefly stale, so
# a one-shot check races and the next POST comes back 409 or reads zeros.
wait_idle() {
  local streak=0
  for _ in $(seq 1 400); do
    if curl -s --max-time 2 http://127.0.0.1:8080/api/state |
       grep -q '"state_fresh":true,"command_active":false'; then
      streak=$((streak+1))
      [[ $streak -ge 3 ]] && return 0
    else
      streak=0
    fi
    sleep 0.2
  done
  return 1
}

CSRF=$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -oP '(?<=name="portal-csrf" content=")[0-9a-f]{64}')
H=(-H "Content-Type: application/json" -H "Origin: http://127.0.0.1:8080"
   -H "Sec-Fetch-Site: same-origin" -H "X-CSRF-Token: $CSRF")

echo "== portal routes =="
check "GET /api/health"   "$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 http://127.0.0.1:8080/api/health)" "200"
check "GET /"             "$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 http://127.0.0.1:8080/)" "200"
check "GET /api/state"    "$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 http://127.0.0.1:8080/api/state)" "200"
check "page has demo buttons" "$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -q 'id="demo-presets"' && echo yes || echo no)" "yes"
check "page has sequence buttons" "$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -q 'id="demo-sequences"' && echo yes || echo no)" "yes"
check "page embeds stream"    "$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -q '/api/rviz/stream' && echo yes || echo no)" "yes"
check "page has box preset"   "$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -q 'box_grasp' && echo yes || echo no)" "yes"
check "page has cross preset" "$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -q 'cross_over' && echo yes || echo no)" "yes"
check "page has Move Both"    "$(curl -s --max-time 5 http://127.0.0.1:8080/ | grep -q 'id=\"both\"' && echo yes || echo no)" "yes"

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

echo "== stream frame rate =="
check "stream delivers at least 30 fps" "$(python3 - <<'PYEOF'
import socket,re,time
try:
    s=socket.create_connection(("127.0.0.1",8080),timeout=15)
    s.sendall(b"GET /api/rviz/stream HTTP/1.1\r\nHost: 127.0.0.1:8080\r\nConnection: close\r\n\r\n")
    buf=b""
    while b"\r\n\r\n" not in buf: buf+=s.recv(4096)
    _,rest=buf.split(b"\r\n\r\n",1)
    frames=0; start=time.time(); s.settimeout(5)
    while time.time()-start < 5.0:
        while b"\r\n\r\n" not in rest:
            c=s.recv(65536)
            if not c: break
            rest+=c
        if b"\r\n\r\n" not in rest: break
        part,body=rest.split(b"\r\n\r\n",1)
        m=re.search(rb"Content-Length: (\d+)",part)
        if not m: break
        n=int(m.group(1))
        while len(body)<n+2:
            c=s.recv(65536)
            if not c: break
            body+=c
        frames+=1; rest=body[n+2:]
    s.close()
    print("yes" if frames/(time.time()-start) >= 30.0 else "no %.1f fps"%(frames/(time.time()-start)))
except Exception as e:
    print("no",e)
PYEOF
)" "yes"

echo "== CLI motion =="
# Start from a known pose: the sweep is order dependent and a previous demo can
# leave the arms somewhere the first check cannot plan from.
climove home >/dev/null 2>&1
check "status"          "$(cli status >/dev/null 2>&1; echo $?)" "0"
check "move-joint"      "$(climove move-joint openarm_left_joint4 0.3 >/dev/null 2>&1; echo $?)" "0"
check "move-paired-tcp" "$(climove move-paired-tcp openarm_body_link0 0.30 0.24 0.40 0.30 -0.24 0.40 >/dev/null 2>&1; echo $?)" "0"
check "mirror"          "$(climove mirror left 0.30 0.22 0.38 >/dev/null 2>&1; echo $?)" "0"
check "centroid"        "$(climove centroid 0.32 0.00 0.42 >/dev/null 2>&1; echo $?)" "0"
check "converge"        "$(climove converge 0.34 0.00 0.40 0.08 >/dev/null 2>&1; echo $?)" "0"
check "converge again"  "$(climove converge 0.34 0.00 0.42 0.06 >/dev/null 2>&1; echo $?)" "0"
check "clap"            "$(climove clap 1 >/dev/null 2>&1; echo $?)" "0"
check "cross"           "$(climove cross 1 >/dev/null 2>&1; echo $?)" "0"
check "home"            "$(climove home >/dev/null 2>&1; echo $?)" "0"
check "pick-place"      "$(climove pick-place >/dev/null 2>&1; echo $?)" "0"

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
wait_idle
cli home >/dev/null 2>&1
wait_idle
check "motion works after release" "$(curl -s -o /dev/null -w '%{http_code}' --max-time 8 -X POST "${H[@]}" -d '{"side":"left","unit":"m","x":0.28,"y":0.20,"z":0.36,"motion_limit_scale":0.8}' http://127.0.0.1:8080/api/v3/move)" "202"

echo "== both arms at once =="
wait_idle
cli home >/dev/null 2>&1
wait_idle
check "dual move accepted" "$(curl -s -o /dev/null -w '%{http_code}' --max-time 10 -X POST "${H[@]}" -d '{"unit":"m","left_x":0.28,"left_y":0.20,"left_z":0.36,"right_x":0.33,"right_y":-0.24,"right_z":0.50,"motion_limit_scale":0.9}' http://127.0.0.1:8080/api/v3/move-both)" "202"
wait_idle
check "both arms reached their own targets" "$(curl -s --max-time 5 http://127.0.0.1:8080/api/state | python3 -c "
import json,sys,math
v=json.load(sys.stdin)
ok = math.dist(v['left'],[0.28,0.20,0.36])<0.01 and math.dist(v['right'],[0.33,-0.24,0.50])<0.01
print('yes' if ok else 'no L=%s R=%s'%(v['left'],v['right']))")" "yes"
wait_idle
check "dual collision rejected" "$(curl -s -o /dev/null -w '%{http_code}' --max-time 10 -X POST "${H[@]}" -d '{"unit":"m","left_x":0.30,"left_y":0.01,"left_z":0.40,"right_x":0.30,"right_y":-0.01,"right_z":0.40,"motion_limit_scale":0.8}' http://127.0.0.1:8080/api/v3/move-both)" "422"
wait_idle
check "dual missing field rejected" "$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 -X POST "${H[@]}" -d '{"unit":"m","left_x":0.28,"left_y":0.20,"left_z":0.36,"right_x":0.33,"right_y":-0.24,"motion_limit_scale":0.9}' http://127.0.0.1:8080/api/v3/move-both)" "400"

echo
echo "SWEEP: $pass passed, $fail failed"
exit $((fail>0))

#!/usr/bin/env bash
set -euo pipefail
portal= web= viewer= license= geckodriver= firefox= check_urdf= urdf= oracle=
while (($#)); do
  case "$1" in
    --portal) portal=$2 ;;
    --web) web=$2 ;;
    --viewer) viewer=$2 ;;
    --license) license=$2 ;;
    --geckodriver) geckodriver=$2 ;;
    --firefox) firefox=$2 ;;
    --check-urdf) check_urdf=$2 ;;
    --urdf) urdf=$2 ;;
    --oracle) oracle=$2 ;;
    *) printf 'unknown viewer browser test argument: %s\n' "$1" >&2; exit 2 ;;
  esac
  shift 2
done
for required in portal web viewer license geckodriver firefox check_urdf urdf oracle; do
  [[ -n ${!required} ]] || { printf 'missing --%s\n' "${required//_/-}" >&2; exit 2; }
done
[[ -x "$portal" && -x "$geckodriver" && -x "$firefox" && -x "$check_urdf" ]] || {
  printf 'Firefox/geckodriver/check_urdf/portal dependency gate failed\n' >&2
  exit 2
}
"$check_urdf" "$urdf"
printf 'viewer browser gate: check_urdf passed\n'

work_root=$(mktemp -d "${PWD}/viewer-browser.XXXXXX")
portal_pid= driver_pid= session_id=
slow_pids=()
partial_fds=()
close_partial_clients() {
  local fd
  for fd in "${partial_fds[@]}"; do
    eval "exec ${fd}>&-" || true
  done
  partial_fds=()
}
open_partial_clients() {
  local client fd
  partial_fds=()
  for client in $(seq 1 16); do
    if exec {fd}<>"/dev/tcp/127.0.0.1/${portal_port}"; then
      partial_fds+=("$fd")
      printf 'POST /api/stop HTTP/1.1\r\nHost: 127.0.0.1:%s\r\nOrigin: http://127.0.0.1:%s\r\nSec-Fetch-Site: same-origin\r\nContent-Type: application/json\r\nX-CSRF-Token: %s\r\nContent-Length: 512\r\nConnection: close\r\n\r\n{' \
        "$portal_port" "$portal_port" "$csrf" >&"$fd" || true
    fi
  done
  [[ ${#partial_fds[@]} -eq 16 ]] || {
    printf 'could not open 16 partial-body clients\n' >&2
    return 1
  }
}
cleanup() {
  close_partial_clients
  if [[ -n "$session_id" ]]; then
    curl -fsS -X DELETE "http://127.0.0.1:${driver_port}/session/${session_id}" >/dev/null 2>&1 || true
  fi
  ((${#slow_pids[@]} == 0)) || kill "${slow_pids[@]}" >/dev/null 2>&1 || true
  [[ -z "$driver_pid" ]] || pkill -TERM -P "$driver_pid" >/dev/null 2>&1 || true
  [[ -z "$driver_pid" ]] || kill "$driver_pid" >/dev/null 2>&1 || true
  [[ -z "$portal_pid" ]] || kill "$portal_pid" >/dev/null 2>&1 || true
  for _ in $(seq 1 20); do
    { [[ -z "$driver_pid" ]] || ! kill -0 "$driver_pid" 2>/dev/null; } &&
      { [[ -z "$portal_pid" ]] || ! kill -0 "$portal_pid" 2>/dev/null; } && break
    sleep .05
  done
  [[ -z "$driver_pid" ]] || ! kill -0 "$driver_pid" 2>/dev/null || kill -KILL "$driver_pid" 2>/dev/null || true
  [[ -z "$portal_pid" ]] || ! kill -0 "$portal_pid" 2>/dev/null || kill -KILL "$portal_pid" 2>/dev/null || true
  [[ -z "$driver_pid" ]] || wait "$driver_pid" >/dev/null 2>&1 || true
  [[ -z "$portal_pid" ]] || wait "$portal_pid" >/dev/null 2>&1 || true
  rm -rf -- "$work_root"
}
trap cleanup EXIT
if [[ "$geckodriver" == /snap/bin/geckodriver &&
      -x /snap/firefox/current/usr/lib/firefox/geckodriver ]]; then
  geckodriver=/snap/firefox/current/usr/lib/firefox/geckodriver
fi
if [[ "$firefox" == /usr/bin/firefox &&
      -x /snap/firefox/current/usr/lib/firefox/firefox ]]; then
  firefox=/snap/firefox/current/usr/lib/firefox/firefox
fi
prefix="$work_root/prefix"
mkdir -p "$prefix/lib/openarm_ik_ros" "$prefix/share/openarm_ik_ros/viewer"
cp -- "$portal" "$prefix/lib/openarm_ik_ros/openarm_portal"
cp -a -- "$web" "$prefix/share/openarm_ik_ros/web"
cp -a -- "$viewer/." "$prefix/share/openarm_ik_ros/viewer/"
cp -- "$license" "$prefix/share/openarm_ik_ros/viewer/openarm_description-LICENSE.txt"

base_port=$((21000 + $$ % 18000))
portal_port=$base_port
driver_port=$((base_port + 1))
"$prefix/lib/openarm_ik_ros/openarm_portal" --port "$portal_port" \
  >"$work_root/portal.log" 2>&1 &
portal_pid=$!
for _ in $(seq 1 100); do
  if curl -fsS -H "Host: 127.0.0.1:${portal_port}" \
    "http://127.0.0.1:${portal_port}/api/health" >/dev/null 2>&1; then break; fi
  kill -0 "$portal_pid" 2>/dev/null || { sed -n '1,120p' "$work_root/portal.log" >&2; exit 1; }
  sleep .05
done
curl -fsS -H "Host: 127.0.0.1:${portal_port}" \
  "http://127.0.0.1:${portal_port}/api/health" >/dev/null
printf 'viewer browser gate: portal healthy on %s\n' "$portal_port"

# The server must retain verified resident bytes even if its copied install prefix drifts.
curl -fsS -H "Host: 127.0.0.1:${portal_port}" \
  "http://127.0.0.1:${portal_port}/web/portal.css" > "$work_root/css-before"
printf 'X' | dd of="$prefix/share/openarm_ik_ros/web/portal.css" \
  bs=1 seek=0 count=1 conv=notrunc status=none
curl -fsS -H "Host: 127.0.0.1:${portal_port}" \
  "http://127.0.0.1:${portal_port}/web/portal.css" > "$work_root/css-after"
cmp "$work_root/css-before" "$work_root/css-after"
curl -fsS -H "Host: 127.0.0.1:${portal_port}" \
  "http://127.0.0.1:${portal_port}/api/health" | jq -e \
  '.healthy == true and .viewer_assets_ready == true' >/dev/null
printf 'viewer browser gate: post-start asset mutation cannot change served bytes or health\n'

# Eight valid nonreading/one-byte-per-second mesh clients may consume only the
# bounded static lane. The independently admitted authenticated stop must remain prompt.
page=$(curl -fsS -H "Host: 127.0.0.1:${portal_port}" "http://127.0.0.1:${portal_port}/")
csrf=$(sed -n 's/.*name="portal-csrf" content="\([0-9a-f]\{64\}\)".*/\1/p' <<<"$page")
[[ ${#csrf} -eq 64 ]] || { printf 'failed to extract portal CSRF token\n' >&2; exit 1; }
for client in $(seq 1 8); do
  curl -sS --limit-rate 1 --max-time 8 -H "Host: 127.0.0.1:${portal_port}" \
    "http://127.0.0.1:${portal_port}/viewer/mesh/link4_symp.stl" \
    >"$work_root/slow-${client}" 2>/dev/null &
  slow_pids+=("$!")
done
sleep .25
stop_begin=$(date +%s%N)
stop_code=$(curl -sS --connect-timeout 0.2 --max-time 0.75 -o "$work_root/stop.json" \
  -w '%{http_code}' -H "Host: 127.0.0.1:${portal_port}" \
  -H "Origin: http://127.0.0.1:${portal_port}" -H 'Sec-Fetch-Site: same-origin' \
  -H 'Content-Type: application/json' -H "X-CSRF-Token: ${csrf}" \
  --data '{}' "http://127.0.0.1:${portal_port}/api/stop")
stop_elapsed_ms=$((($(date +%s%N) - stop_begin) / 1000000))
[[ "$stop_code" == 200 && "$stop_elapsed_ms" -lt 750 ]] || {
  printf 'backpressured stop failed: code=%s elapsed_ms=%s\n' "$stop_code" "$stop_elapsed_ms" >&2
  exit 1
}
kill "${slow_pids[@]}" >/dev/null 2>&1 || true
for slow_pid in "${slow_pids[@]}"; do
  wait "$slow_pid" >/dev/null 2>&1 || true
done
slow_pids=()
printf 'viewer browser gate: eight static clients left stop available in %s ms\n' "$stop_elapsed_ms"

# Sixteen syntactically valid requests that withhold their declared bodies must
# neither occupy worker threads nor prevent the next authenticated stop request.
baseline_fds=$(find "/proc/${portal_pid}/fd" -mindepth 1 -maxdepth 1 -type l | wc -l)
baseline_threads=$(find "/proc/${portal_pid}/task" -mindepth 1 -maxdepth 1 -type d | wc -l)
open_partial_clients
sleep .15
stop_begin=$(date +%s%N)
stop_code=$(curl -sS --connect-timeout 0.2 --max-time 0.75 -o "$work_root/intake-stop.json" \
  -w '%{http_code}' -H "Host: 127.0.0.1:${portal_port}" \
  -H "Origin: http://127.0.0.1:${portal_port}" -H 'Sec-Fetch-Site: same-origin' \
  -H 'Content-Type: application/json' -H "X-CSRF-Token: ${csrf}" \
  --data '{}' "http://127.0.0.1:${portal_port}/api/stop")
intake_stop_elapsed_ms=$((($(date +%s%N) - stop_begin) / 1000000))
[[ "$stop_code" == 200 && "$intake_stop_elapsed_ms" -lt 750 ]] || {
  printf 'partial-body stop failed: code=%s elapsed_ms=%s\n' \
    "$stop_code" "$intake_stop_elapsed_ms" >&2
  exit 1
}
sleep .55
expired_fds=$(find "/proc/${portal_pid}/fd" -mindepth 1 -maxdepth 1 -type l | wc -l)
expired_threads=$(find "/proc/${portal_pid}/task" -mindepth 1 -maxdepth 1 -type d | wc -l)
[[ "$expired_fds" -le "$baseline_fds" && "$expired_threads" -eq "$baseline_threads" ]] || {
  printf 'partial-body deadline leaked resources: fds=%s/%s threads=%s/%s\n' \
    "$expired_fds" "$baseline_fds" "$expired_threads" "$baseline_threads" >&2
  exit 1
}
close_partial_clients
printf 'viewer browser gate: sixteen partial bodies left stop available in %s ms; deadline reclaimed all server resources\n' \
  "$intake_stop_elapsed_ms"

mkdir -p "$work_root/profiles"
"$geckodriver" --host 127.0.0.1 --port "$driver_port" --log debug \
  --profile-root "$work_root/profiles" \
  >"$work_root/geckodriver.log" 2>&1 &
driver_pid=$!
for _ in $(seq 1 40); do
  curl -fsS --connect-timeout 0.2 --max-time 0.5 \
    "http://127.0.0.1:${driver_port}/status" >/dev/null 2>&1 && break
  kill -0 "$driver_pid" 2>/dev/null || { sed -n '1,120p' "$work_root/geckodriver.log" >&2; exit 1; }
  sleep .05
done
curl -fsS --connect-timeout 0.2 --max-time 0.5 \
  "http://127.0.0.1:${driver_port}/status" >/dev/null
printf 'viewer browser gate: geckodriver ready on %s\n' "$driver_port"
session_payload=$(jq -nc --arg binary "$firefox" \
  '{capabilities:{alwaysMatch:{browserName:"firefox","moz:firefoxOptions":{binary:$binary,args:["-headless","-no-remote"]}}}}')
if ! session_response=$(curl -fsS --connect-timeout 2 --max-time 30 \
  -H 'Content-Type: application/json' -d "$session_payload" \
  "http://127.0.0.1:${driver_port}/session"); then
  sed -n '1,200p' "$work_root/geckodriver.log" >&2
  exit 1
fi
session_id=$(jq -er '.value.sessionId' <<<"$session_response")
printf 'viewer browser gate: Firefox session created\n'
curl -fsS -H 'Content-Type: application/json' -d '{"script":30000,"pageLoad":30000}' \
  "http://127.0.0.1:${driver_port}/session/${session_id}/timeouts" >/dev/null
curl -fsS -H 'Content-Type: application/json' \
  -d "$(jq -nc --arg url "http://127.0.0.1:${portal_port}/" '{url:$url}')" \
  "http://127.0.0.1:${driver_port}/session/${session_id}/url" >/dev/null

ready=false
for _ in $(seq 1 300); do
  value=$(curl -fsS --connect-timeout 2 --max-time 5 -H 'Content-Type: application/json' \
    -d '{"script":"return Boolean(window.__openarmViewerTest && window.__openarmViewerTest.ready());","args":[]}' \
    "http://127.0.0.1:${driver_port}/session/${session_id}/execute/sync" | jq -r '.value')
  if [[ "$value" == true ]]; then ready=true; break; fi
  sleep .05
done
if [[ "$ready" != true ]]; then
  curl -fsS -H 'Content-Type: application/json' \
    -d '{"script":"return {overlay:document.getElementById(\"viewer-overlay\").textContent,metrics:document.getElementById(\"viewer-metrics\").textContent};","args":[]}' \
    "http://127.0.0.1:${driver_port}/session/${session_id}/execute/sync" >&2 || true
  sed -n '1,160p' "$work_root/geckodriver.log" >&2
  exit 1
fi
oracle_payload=$(jq -Rs '{script:.,args:[]}' < "$oracle")
oracle_response=$(curl -fsS --connect-timeout 2 --max-time 35 \
  -H 'Content-Type: application/json' -d "$oracle_payload" \
  "http://127.0.0.1:${driver_port}/session/${session_id}/execute/async")
jq -e '.value.ok == true' <<<"$oracle_response" >/dev/null || {
  jq . <<<"$oracle_response" >&2
  exit 1
}
jq -c '.value' <<<"$oracle_response"

# Partial request bodies must also be cancelled during process shutdown instead
# of extending SIGTERM by an intake timeout or a blocked worker join.
open_partial_clients
sleep .15
term_begin=$(date +%s%N)
kill "$portal_pid"
wait "$portal_pid"
term_elapsed_ms=$((($(date +%s%N) - term_begin) / 1000000))
[[ "$term_elapsed_ms" -lt 1000 ]] || {
  printf 'partial-body SIGTERM cleanup took %s ms\n' "$term_elapsed_ms" >&2
  exit 1
}
portal_pid=
close_partial_clients
printf 'viewer browser gate: partial-body SIGTERM cleanup completed in %s ms\n' \
  "$term_elapsed_ms"

# The same on-disk mutation must fail the next startup's pinned hash validation.
corrupt_port=$((driver_port + 1))
"$prefix/lib/openarm_ik_ros/openarm_portal" --port "$corrupt_port" \
  >"$work_root/corrupt-portal.log" 2>&1 &
portal_pid=$!
corrupt_code=000
for _ in $(seq 1 100); do
  corrupt_code=$(curl -sS --connect-timeout 0.2 --max-time 0.5 \
    -o "$work_root/corrupt-health.json" -w '%{http_code}' \
    -H "Host: 127.0.0.1:${corrupt_port}" \
    "http://127.0.0.1:${corrupt_port}/api/health" 2>/dev/null || true)
  [[ "$corrupt_code" != 000 ]] && break
  sleep .05
done
[[ "$corrupt_code" == 503 ]] && jq -e \
  '.healthy == false and .viewer_assets_ready == false and (.reason | contains("SHA-256"))' \
  "$work_root/corrupt-health.json" >/dev/null || {
  printf 'corrupted startup did not fail closed: code=%s\n' "$corrupt_code" >&2
  sed -n '1,120p' "$work_root/corrupt-portal.log" >&2
  exit 1
}
printf 'viewer browser gate: mutated copied prefix fails SHA-256 on next startup\n'

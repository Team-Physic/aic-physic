#!/usr/bin/env bash

set -uo pipefail

child_pid=""

group_alive() {
  local state
  [[ -n "${child_pid}" ]] || return 1
  while read -r state; do
    if [[ "${state}" != Z* ]]; then
      return 0
    fi
  done < <(ps -o stat= -g "${child_pid}" 2>/dev/null)
  return 1
}

stop_group() {
  if ! group_alive; then
    return
  fi

  echo "[shutdown] SIGKILL -> process group ${child_pid}" >&2
  kill -KILL -- "-${child_pid}" 2>/dev/null || true
}

on_signal() {
  local status="$1"
  trap - INT TERM HUP EXIT
  stop_group
  wait "${child_pid}" 2>/dev/null || true
  exit "${status}"
}

trap 'on_signal 130' INT
trap 'on_signal 143' TERM
trap 'on_signal 129' HUP
trap 'status=$?; stop_group; exit "${status}"' EXIT

# 별도 session/process group에서 router, ros2 launch, Gazebo를 함께 시작한다.
setsid /entrypoint.sh "$@" &
child_pid=$!

status=0
while group_alive; do
  sleep 0.2
done
wait "${child_pid}" 2>/dev/null || status=$?

trap - INT TERM EXIT
stop_group
exit "${status}"

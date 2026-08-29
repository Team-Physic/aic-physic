#!/usr/bin/env bash

set -uo pipefail

child_pid=""
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(dirname -- "${script_dir}")"

usage() {
  echo "usage: $0 --seed N --num-trials N -- [launch_arg:=value ...]" >&2
}

prepare_seeded_config() {
  local seed="$1"
  local num_trials="$2"
  local output_dir="${repo_dir}/ws_aic/results/eval_configs"
  local timestamp
  timestamp="$(date +%Y%m%d_%H%M%S_%N)"
  generated_config="${output_dir}/${timestamp}_seed${seed}_trials${num_trials}.yaml"

  if ! python3 "${script_dir}/generate_aic_eval_config.py" \
    --template "${repo_dir}/ws_aic/src/aic/aic_engine/config/eval_config.yaml" \
    --output "${generated_config}" \
    --seed "${seed}" \
    --num-trials "${num_trials}"; then
    echo "[error] failed to generate seeded AIC evaluation config" >&2
    exit 1
  fi
  echo "[scenario] seed=${seed}, trials=${num_trials}, config=${generated_config}"
}

launch_args=("$@")
if [[ "${1:-}" == "--seed" || "${1:-}" == "--num-trials" ]]; then
  seed=""
  num_trials=""
  separator_found="false"
  while (($#)); do
    case "$1" in
      --seed)
        [[ $# -ge 2 ]] || { usage; exit 2; }
        seed="$2"
        shift 2
        ;;
      --num-trials)
        [[ $# -ge 2 ]] || { usage; exit 2; }
        num_trials="$2"
        shift 2
        ;;
      --)
        shift
        separator_found="true"
        break
        ;;
      *)
        echo "[error] unknown runner option before --: $1" >&2
        usage
        exit 2
        ;;
    esac
  done

  [[ -n "${seed}" && -n "${num_trials}" ]] || { usage; exit 2; }
  [[ "${separator_found}" == "true" ]] || {
    echo "[error] -- is required before launch arguments" >&2
    usage
    exit 2
  }
  [[ "${seed}" =~ ^(0|[1-9][0-9]{0,9})$ ]] && ((10#${seed} <= 4294967295)) || {
    echo "[error] --seed must be an integer in 0..4294967295" >&2
    exit 2
  }
  [[ "${num_trials}" =~ ^[1-9][0-9]*$ ]] || {
    echo "[error] --num-trials must be a positive integer" >&2
    exit 2
  }

  launch_args=("$@")
  for arg in "${launch_args[@]}"; do
    case "${arg}" in
      num_trials:=*|aic_engine_config_file:=*)
        echo "[error] ${arg%%:=*} is managed by --seed mode" >&2
        exit 2
        ;;
    esac
  done

  prepare_seeded_config "${seed}" "${num_trials}"
  launch_args+=(
    "num_trials:=${num_trials}"
    "aic_engine_config_file:=${generated_config}"
  )
fi

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
setsid /entrypoint.sh "${launch_args[@]}" &
child_pid=$!

status=0
while group_alive; do
  sleep 0.2
done
wait "${child_pid}" 2>/dev/null || status=$?

trap - INT TERM EXIT
stop_group
exit "${status}"

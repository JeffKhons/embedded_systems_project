#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
controller_pid=""
camera_pid=""
module_loaded_by_script=0
mode="hardware"
start_camera=0

for argument in "$@"; do
    case "$argument" in
        --mock) mode="mock" ;;
        --with-camera) start_camera=1 ;;
        *) echo "Usage: $0 [--mock] [--with-camera]" >&2; exit 2 ;;
    esac
done

cleanup() {
    trap - INT TERM EXIT
    [[ -n "$camera_pid" ]] && kill "$camera_pid" 2>/dev/null || true
    [[ -n "$controller_pid" ]] && kill "$controller_pid" 2>/dev/null || true
    [[ -n "$camera_pid" ]] && wait "$camera_pid" 2>/dev/null || true
    [[ -n "$controller_pid" ]] && wait "$controller_pid" 2>/dev/null || true
    if [[ "$module_loaded_by_script" -eq 1 ]]; then
        sudo rmmod dual_stepper || true
    fi
}
trap cleanup INT TERM EXIT

cd "$project_dir"
make app tools

controller_command=(./build/wall_robot_controller)
if [[ "$mode" == "mock" ]]; then
    controller_command+=(--mock)
else
    make driver
    if ! grep -q '^dual_stepper ' /proc/modules; then
        sudo insmod driver/dual_stepper.ko
        module_loaded_by_script=1
    fi
    controller_command=(sudo ./build/wall_robot_controller)
fi

"${controller_command[@]}" &
controller_pid=$!

if [[ "$start_camera" -eq 1 ]]; then
    make vision
    ./build/camera_app &
    camera_pid=$!
fi

echo "controller PID: $controller_pid ($mode mode)"
[[ -n "$camera_pid" ]] && echo "camera PID: $camera_pid"
echo "Press Ctrl+C to stop."
wait "$controller_pid"

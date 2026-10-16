#!/usr/bin/env bash
set -euo pipefail

RUNTIME_DIR="${TMPDIR:-/tmp}/px4-scenarios/02_rocket_two_axis_pod"

declare -A seen=()
pids=()
add_pid() {
	local pid=$1
	[[ $pid =~ ^[0-9]+$ ]] || return 0
	[[ -n ${seen[$pid]:-} ]] && return 0
	seen[$pid]=1
	pids+=("$pid")
}

if [[ -d $RUNTIME_DIR/pids ]]; then
	while IFS= read -r pid_file; do
		pid=$(<"$pid_file")
		kill -0 "$pid" 2>/dev/null && add_pid "$pid"
	done < <(find "$RUNTIME_DIR/pids" -type f -name '*.pid' -print)
fi

while IFS= read -r pid; do add_pid "$pid"; done < <(
	{
		pgrep -f '[/]build/px4_sitl[^/]*/bin/px4' || true
		pgrep -f '(^|[ /])gz sim([ ]|$)' || true
		pgrep -f '[g]imbal_bridge[.]py.*--world geo_32_217857_119_105878' || true
		pgrep -f '[c]ollision_manager[.]py.*--aircraft-count' || true
		pgrep -f '[t]arget_manager[.]py.*--world geo_32_217857_119_105878' || true
	} | sort -nu
)

if ((${#pids[@]} == 0)); then
	echo "当前没有 PX4 SITL 场景进程。"
	rm -rf -- "$RUNTIME_DIR"
	exit 0
fi

echo "停止当前 PX4 SITL/Gazebo/吊仓网关/靶机管理进程：${pids[*]}"
for pid in "${pids[@]}"; do
	kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
done

for _ in {1..50}; do
	remaining=0
	for pid in "${pids[@]}"; do
		kill -0 "$pid" 2>/dev/null && remaining=1
	done
	((remaining == 0)) && break
	sleep 0.1
done

for pid in "${pids[@]}"; do
	if kill -0 "$pid" 2>/dev/null; then
		kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
	fi
done

rm -rf -- "$RUNTIME_DIR"
echo "当前 PX4 SITL、Gazebo、吊仓网关、靶机与碰撞致损服务已停止。共享 RTSP 中继会在无视频路后自动退出。"

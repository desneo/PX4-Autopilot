#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PX4_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
PX4_ETC="${PX4_DIR}/build/px4_sitl_default/etc"
DYT_UDP_BRIDGE_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4-dyt_udp_bridge"
RUNTIME_DIR="${TMPDIR:-/tmp}/px4-scenarios/02_rocket_two_axis_pod"
ASSETS_ROOT="${PX4_GAZEBO_ASSETS_ROOT:-${PX4_DIR}/../px4-gazebo-assets}"
ASSETS_URL="${PX4_GAZEBO_ASSETS_URL:-https://github.com/desneo/px4-gazebo-assets.git}"
BRIDGE_PYTHON="${GIMBAL_PYTHON:-python3}"
SCENARIO_GUI_DIR="${SCRIPT_DIR}/gui"
COUNT=1
GUI=0
ENCODER=nvenc
ADVERTISE_HOST=""
WORLD=geo_32_217857_119_105878
PARTITION=px4_scenario_02_rocket_two_axis_pod
RTSP_PORT=8555
TARGET_QGC_ADDRESS=127.0.0.1:14550
TARGET_MAVLINK_SYSID=200
TRACKING_BACKEND=sim_truth

usage() {
	cat <<'EOF'
用法：start.sh [-n 架数] [--gui] [--advertise-host 地址] [--encoder nvenc] [--tracking-backend sim_truth|csrt]

在 geo_32_217857_119_105878 world 中启动火箭穿越机与两轴吊仓组合模型。
默认启动 1 架、不开启 GUI，启动后等待 MAVSDK 和吊仓地面站控制。

选项：
  -n, --count COUNT       飞机数量，1～10（默认：1）
      --gui               启动 Gazebo GUI
      --advertise-host IP 打印给远程客户端使用的本机地址
      --encoder TYPE      吊仓视频编码器：nvenc（默认：nvenc）
      --tracking-backend TYPE
                           单目标跟踪后端：sim_truth（默认）或 csrt
      --target-qgc-address HOST:PORT
                           轻量靶机 MAVLink 目标（默认：127.0.0.1:14550）
      --target-sysid ID   QGC 中的靶机 SYSID（默认：200）
  -h, --help              显示帮助

环境变量：
  PX4_GAZEBO_ASSETS_ROOT  资产仓路径，默认 ../px4-gazebo-assets
  PX4_GAZEBO_ASSETS_URL   自动下载使用的仓库地址
  GIMBAL_PYTHON           吊仓网关使用的 Python 解释器
EOF
}

require_value() {
	if (($# < 2)) || [[ -z $2 || $2 == --* ]]; then
		echo "ERROR: 参数 $1 缺少值" >&2
		exit 2
	fi
}

while (($# > 0)); do
	case "$1" in
		-n|--count)
			require_value "$@"
			COUNT=$2
			shift 2
			;;
		--gui)
			GUI=1
			shift
			;;
		--advertise-host)
			require_value "$@"
			ADVERTISE_HOST=$2
			shift 2
			;;
		--encoder)
			require_value "$@"
			ENCODER=$2
			shift 2
			;;
		--tracking-backend)
			require_value "$@"
			TRACKING_BACKEND=$2
			shift 2
			;;
		--target-qgc-address)
			require_value "$@"
			TARGET_QGC_ADDRESS=$2
			shift 2
			;;
		--target-sysid)
			require_value "$@"
			TARGET_MAVLINK_SYSID=$2
			shift 2
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			echo "ERROR: 未知参数：$1" >&2
			usage >&2
			exit 2
			;;
	esac
done

if [[ ! $COUNT =~ ^[0-9]+$ ]] || ((COUNT < 1 || COUNT > 10)); then
	echo "ERROR: 架数必须是 1～10 的整数" >&2
	exit 2
fi
if [[ $ENCODER != nvenc ]]; then
	echo "ERROR: 最新资产仓的 --encoder 只接受 nvenc" >&2
	exit 2
fi
if [[ $TRACKING_BACKEND != sim_truth && $TRACKING_BACKEND != csrt ]]; then
	echo "ERROR: --tracking-backend 只接受 sim_truth 或 csrt" >&2
	exit 2
fi
if [[ $ADVERTISE_HOST == *[[:space:]]* ]]; then
	echo "ERROR: --advertise-host 不能包含空白字符" >&2
	exit 2
fi
if [[ $TARGET_QGC_ADDRESS == *[[:space:]]* || $TARGET_QGC_ADDRESS != *:* ]]; then
	echo "ERROR: --target-qgc-address 格式应为 HOST:PORT" >&2
	exit 2
fi
if [[ ! $TARGET_MAVLINK_SYSID =~ ^[0-9]+$ ]] \
	|| ((TARGET_MAVLINK_SYSID < 1 || TARGET_MAVLINK_SYSID > 254)); then
	echo "ERROR: --target-sysid 必须是 1～254 的整数" >&2
	exit 2
fi
if ((TARGET_MAVLINK_SYSID <= COUNT)); then
	echo "ERROR: 靶机 SYSID ${TARGET_MAVLINK_SYSID} 与普通飞机 SYSID 1～${COUNT} 冲突" >&2
	exit 2
fi

if pgrep -f '[/]build/px4_sitl[^/]*/bin/px4' >/dev/null \
	|| pgrep -f '(^|[ /])gz sim([ ]|$)' >/dev/null \
	|| pgrep -f '[g]imbal_bridge[.]py' >/dev/null \
	|| pgrep -f '[c]ollision_manager[.]py.*--aircraft-count' >/dev/null \
	|| pgrep -f '[t]arget_manager[.]py.*--world geo_32_217857_119_105878' >/dev/null \
	|| pgrep -f '[r]adar_manager[.]py.*--world geo_32_217857_119_105878' >/dev/null; then
	echo "ERROR: 当前已有 PX4 SITL、Gazebo 或本场景吊仓网关在运行。" >&2
	echo "请先执行 ${SCRIPT_DIR}/stop.sh。" >&2
	exit 1
fi

ASSETS_ROOT=$(realpath -m -- "$ASSETS_ROOT")
if [[ ! -e $ASSETS_ROOT ]]; then
	echo "未找到资产仓，开始下载最新 main：${ASSETS_URL}"
	mkdir -p -- "$(dirname -- "$ASSETS_ROOT")"
	git clone --branch main --single-branch "$ASSETS_URL" "$ASSETS_ROOT" || {
		echo "ERROR: 资产仓下载失败：${ASSETS_URL}" >&2
		exit 1
	}
elif [[ ! -d $ASSETS_ROOT/.git ]]; then
	echo "ERROR: 资产路径不是 Git 仓库：${ASSETS_ROOT}" >&2
	exit 1
fi

if [[ $(git -C "$ASSETS_ROOT" branch --show-current) != main ]]; then
	echo "ERROR: 资产仓当前不是 main 分支：${ASSETS_ROOT}" >&2
	exit 1
fi
echo "检查资产仓远程 main：${ASSETS_ROOT}"
git -C "$ASSETS_ROOT" fetch --quiet origin main || {
	echo "ERROR: 无法获取资产仓 origin/main，不能确认本地版本是否过期。" >&2
	exit 1
}
if ! git -C "$ASSETS_ROOT" merge-base --is-ancestor origin/main HEAD; then
	echo "ERROR: 资产仓本地 main 未包含最新 origin/main，请先更新本地提交。" >&2
	exit 1
fi
if [[ -n $(git -C "$ASSETS_ROOT" status --porcelain) ]]; then
	echo "资产仓存在未提交修改；本地提交未落后 origin/main，继续使用当前工作区。"
fi
ASSETS_REV=$(git -C "$ASSETS_ROOT" rev-parse --short=12 HEAD)
echo "资产仓版本：${ASSETS_REV}"

WORLD_SOURCE="${ASSETS_ROOT}/worlds/${WORLD}/world.sdf"
GENERATOR="${ASSETS_ROOT}/assembledmodels/rocket_two_axis_pod/tools/generate_dynamic_model.py"
CONTROLLER_DIR="${ASSETS_ROOT}/models/gimbals/two_axis_pod/controller"
BRIDGE_SERVICE="${CONTROLLER_DIR}/service"
for required in "$WORLD_SOURCE" "$GENERATOR" "$CONTROLLER_DIR/CMakeLists.txt" \
	"$BRIDGE_SERVICE/gimbal_bridge.py" \
	"$ASSETS_ROOT/worlds/${WORLD}/metadata.json" \
	"$ASSETS_ROOT/models/vehicles/xiaomotuo/model.sdf" \
	"$ASSETS_ROOT/models/vehicles/xiaobianfu/model.sdf" \
	"$SCRIPT_DIR/collision_manager.py" \
	"$SCRIPT_DIR/target_manager.py" \
	"$SCRIPT_DIR/radar_manager.py"; do
	[[ -f $required ]] || { echo "ERROR: 最新资产仓缺少必要文件：${required}" >&2; exit 1; }
done

for command in gz setsid timeout git cmake make find; do
	command -v "$command" >/dev/null || { echo "ERROR: 缺少命令：${command}" >&2; exit 1; }
done

if [[ ! -x $PX4_BIN || ! -x $DYT_UDP_BRIDGE_BIN || ! -d $PX4_ETC ]]; then
	echo "PX4 SITL 尚未构建，开始构建 px4_sitl_default。"
	make -C "$PX4_DIR" px4_sitl_default
fi

echo "增量构建两轴吊仓控制插件。"
cmake -S "$CONTROLLER_DIR" -B "$CONTROLLER_DIR/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$CONTROLLER_DIR/build" -j"$(nproc)"

GUI_PLUGIN_DIR="${ASSETS_ROOT}/worlds/${WORLD}/gui"
echo "增量构建 02 场景碰撞监控与靶机插件。"
if ((GUI)); then
	cmake -S "$SCENARIO_GUI_DIR" -B "$SCENARIO_GUI_DIR/build" \
		-DCMAKE_BUILD_TYPE=Release -DSCENARIO02_BUILD_GUI=ON -DBUILD_TESTING=OFF
	cmake --build "$SCENARIO_GUI_DIR/build" -j"$(nproc)"
else
	cmake -S "$SCENARIO_GUI_DIR" -B "$SCENARIO_GUI_DIR/build" \
		-DCMAKE_BUILD_TYPE=Release -DSCENARIO02_BUILD_GUI=OFF -DBUILD_TESTING=OFF
	cmake --build "$SCENARIO_GUI_DIR/build" --target CollisionMonitor -j"$(nproc)"
fi
if ((GUI)); then
	echo "增量构建 world 的 Gazebo GUI 插件。"
	cmake -S "$GUI_PLUGIN_DIR" -B "$GUI_PLUGIN_DIR/build" -DCMAKE_BUILD_TYPE=Release
	cmake --build "$GUI_PLUGIN_DIR/build" -j"$(nproc)"
fi

python3 -c 'import numpy, gz.transport13, gz.msgs10.stringmsg_pb2, gz.msgs10.pose_v_pb2'
"$BRIDGE_PYTHON" -c 'import gi, gz.transport13; gi.require_version("Gst", "1.0"); from gi.repository import Gst'

if [[ -z $ADVERTISE_HOST ]]; then
	# 不使用 `ip route get` 查询公网地址：代理软件可能通过策略路由
	# 返回 mihomo 等虚拟接口的 198.18.0.0/15 地址。改为读取主默认
	# 路由接口上的全局 IPv4，例如 enp130s0 上的 192.168.1.56。
	default_interface=$(ip -o -4 route show to default 2>/dev/null \
		| awk 'NR == 1 { for (i = 1; i <= NF; i++) if ($i == "dev") { print $(i + 1); exit } }' \
		|| true)
	if [[ -n $default_interface ]]; then
		ADVERTISE_HOST=$(ip -o -4 addr show dev "$default_interface" scope global 2>/dev/null \
			| awk 'NR == 1 { split($4, address, "/"); print address[1] }' || true)
	fi
	[[ -n $ADVERTISE_HOST ]] || ADVERTISE_HOST=127.0.0.1
fi

python3 - "$COUNT" <<'PY_PORTS'
import socket
import sys

count = int(sys.argv[1])
sockets = []
try:
    for port in range(8770, 8770 + count):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sockets.append(sock)
        try:
            sock.bind(("0.0.0.0", port))
        except OSError as error:
            raise SystemExit(f"ERROR: 吊仓 UDP 端口 {port} 已被使用：{error}")
finally:
    for sock in sockets:
        sock.close()
PY_PORTS

rm -rf -- "$RUNTIME_DIR"
mkdir -p -- "$RUNTIME_DIR/models" "$RUNTIME_DIR/logs" "$RUNTIME_DIR/pids" "$RUNTIME_DIR/target"
printf '%s\n' "$COUNT" > "$RUNTIME_DIR/count"
printf '%s\n' "$ASSETS_ROOT" > "$RUNTIME_DIR/assets_root"
printf '%s\n' "$ASSETS_REV" > "$RUNTIME_DIR/assets_revision"

model_args=()
for ((i = 0; i < COUNT; i++)); do
	aircraft_id=$((i + 1))
	generated="${RUNTIME_DIR}/generated/aircraft_${aircraft_id}/rocket_two_axis_pod"
	python3 "$GENERATOR" --output-dir "$generated" --topic-prefix "/uav_${aircraft_id}/pod"
	model_args+=(--model "$aircraft_id" "$generated/model.sdf")
done

WORLD_FILE="${RUNTIME_DIR}/world.sdf"
prepare_world_command=(python3 "$SCRIPT_DIR/prepare_world.py"
	--source-world "$WORLD_SOURCE" --output "$WORLD_FILE"
	--tracking-backend "$TRACKING_BACKEND" "${model_args[@]}")
((GUI)) && prepare_world_command+=(--target-control)
"${prepare_world_command[@]}"
gz sdf -k "$WORLD_FILE"

cp -R "$PX4_ETC" "$RUNTIME_DIR/etc"
rm -f "$RUNTIME_DIR/etc/init.d-posix/airframes/"*_gz_rocket_two_axis_pod
mapfile -t generated_airframes < <(find \
	"$RUNTIME_DIR/generated/aircraft_1/rocket_two_axis_pod/px4" -maxdepth 1 -type f \
	-name '*_gz_rocket_two_axis_pod' -print)
if ((${#generated_airframes[@]} != 1)); then
	echo "ERROR: 最新资产仓应生成一个 rocket_two_axis_pod PX4 机体文件，实际为 ${#generated_airframes[@]} 个。" >&2
	exit 1
fi
runtime_airframe="$RUNTIME_DIR/etc/init.d-posix/airframes/$(basename -- "${generated_airframes[0]}")"
cp "${generated_airframes[0]}" "$runtime_airframe"
if ! grep -q '^param set-default EKF2_HGT_REF ' "$runtime_airframe"; then
	echo "ERROR: 最新资产仓生成的机体文件缺少 EKF2_HGT_REF 配置。" >&2
	exit 1
fi
# 本 world 的 GPS 海拔约为 4.2 m，而气压高度原点约为 0 m。使用 GPS
# 高度参考，确保全局高度有效，Mission 航线模式可以通过解锁检查。
sed -i 's/^param set-default EKF2_HGT_REF .*/param set-default EKF2_HGT_REF 1/' "$runtime_airframe"

TARGET_SYSTEM_PLUGIN_DIR="${SCENARIO_GUI_DIR}/build/target-system-plugins"

RESOURCE_PATH="${RUNTIME_DIR}/models:${ASSETS_ROOT}/assembledmodels:${ASSETS_ROOT}/models/vehicles:${ASSETS_ROOT}/models/gimbals${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
SYSTEM_PLUGIN_PATH="${CONTROLLER_DIR}/build/lib"
SYSTEM_PLUGIN_PATH="${SYSTEM_PLUGIN_PATH}:${TARGET_SYSTEM_PLUGIN_DIR}"
SYSTEM_PLUGIN_PATH="${SYSTEM_PLUGIN_PATH}${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
SERVER_CONFIG_PATH="${ASSETS_ROOT}/models/gimbals/two_axis_pod/scripts/server.config"
GUI_PLUGIN_PATH="${GUI_PLUGIN_DIR}/build:${SCENARIO_GUI_DIR}/build${GZ_GUI_PLUGIN_PATH:+:${GZ_GUI_PLUGIN_PATH}}"
export GZ_PARTITION="$PARTITION"
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="$RESOURCE_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$SYSTEM_PLUGIN_PATH"
export GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG_PATH"
((GUI)) && export GZ_GUI_PLUGIN_PATH="$GUI_PLUGIN_PATH"

# WSL 的 Mesa 可能默认选择 llvmpipe；GUI 和无窗口相机均使用 D3D12。
if [[ -e /dev/dxg ]] && grep -qi microsoft /proc/sys/kernel/osrelease; then
	export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
	export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
	echo "WSL 渲染配置：GALLIUM_DRIVER=${GALLIUM_DRIVER}，适配器=${MESA_D3D12_DEFAULT_ADAPTER_NAME}"
fi

start_process() {
	local name=$1
	shift
	setsid "$@" >"${RUNTIME_DIR}/logs/${name}.log" 2>&1 </dev/null &
	local pid=$!
	printf '%s\n' "$pid" > "${RUNTIME_DIR}/pids/${name}.pid"
}

wait_for_log() {
	local name=$1
	local pattern=$2
	local timeout_seconds=$3
	local pid log_file
	pid=$(<"${RUNTIME_DIR}/pids/${name}.pid")
	log_file="${RUNTIME_DIR}/logs/${name}.log"
	for ((attempt = 0; attempt < timeout_seconds * 2; attempt++)); do
		grep -q -- "$pattern" "$log_file" 2>/dev/null && return 0
		if ! kill -0 "$pid" 2>/dev/null; then
			echo "ERROR: ${name} 启动期间退出" >&2
			tail -n 50 "$log_file" >&2 || true
			return 1
		fi
		sleep 0.5
	done
	echo "ERROR: 等待 ${name} 就绪超时" >&2
	tail -n 50 "$log_file" >&2 || true
	return 1
}

wait_for_gz_message() {
	local name=$1
	local topic=$2
	local timeout_seconds=$3
	local deadline=$((SECONDS + timeout_seconds))
	local attempt=0
	local status=0
	local log_file="${RUNTIME_DIR}/logs/${name}.log"
	: > "$log_file"
	while ((SECONDS < deadline)); do
		attempt=$((attempt + 1))
		if timeout 10 gz topic -e -t "$topic" -n 1 >>"$log_file" 2>&1; then
			return 0
		else
			status=$?
		fi
		printf '第 %d 次等待失败（exit=%d），继续重试：%s\n' \
			"$attempt" "$status" "$topic" >> "$log_file"
		sleep 0.5
	done
	return 1
}

FAILED=1
cleanup_on_failure() {
	if ((FAILED)); then
		echo "场景启动失败，清理已启动的进程。" >&2
		bash "$SCRIPT_DIR/stop.sh" >/dev/null 2>&1 || true
	fi
}
trap cleanup_on_failure EXIT

gazebo_command=(env -u DISPLAY GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1
	GZ_SIM_RESOURCE_PATH="$RESOURCE_PATH" GZ_SIM_SYSTEM_PLUGIN_PATH="$SYSTEM_PLUGIN_PATH"
	GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG_PATH"
	gz sim -r -s --headless-rendering -v2 "$WORLD_FILE")
printf 'Gazebo command: '
printf '%q ' "${gazebo_command[@]}"
printf '\n'
start_process gazebo "${gazebo_command[@]}"

for ((i = 0; i < COUNT; i++)); do
	aircraft_id=$((i + 1))
	entity="rocket_two_axis_pod_${aircraft_id}"
	imu_topic="/world/${WORLD}/model/${entity}/link/base_link/sensor/imu_sensor/imu"
	if ! wait_for_gz_message "imu_${aircraft_id}" "$imu_topic" 120; then
		echo "ERROR: 飞机 ${aircraft_id} 的 IMU 未就绪：${imu_topic}" >&2
		tail -n 30 "${RUNTIME_DIR}/logs/gazebo.log" >&2 || true
		exit 1
	fi
done

if ((GUI)); then
	target_manager_command=(python3 -u "$SCRIPT_DIR/target_manager.py"
		--world "$WORLD" --runtime-dir "$RUNTIME_DIR" --assets-root "$ASSETS_ROOT"
		--qgc-address "$TARGET_QGC_ADDRESS"
		--mavlink-system-id "$TARGET_MAVLINK_SYSID")
	printf '靶机管理服务 command: '
	printf '%q ' "${target_manager_command[@]}"
	printf '\n'
	start_process target_manager "${target_manager_command[@]}"
	wait_for_log target_manager '靶机管理服务已就绪' 15 || exit 1

	radar_manager_command=(python3 -u "$SCRIPT_DIR/radar_manager.py"
		--world "$WORLD" --metadata "$ASSETS_ROOT/worlds/$WORLD/metadata.json")
	printf '雷达管理服务 command: '
	printf '%q ' "${radar_manager_command[@]}"
	printf '\n'
	start_process radar_manager "${radar_manager_command[@]}"
	wait_for_log radar_manager '雷达管理服务已就绪' 15 || exit 1

	printf 'Gazebo GUI command: '
	printf '%q ' env GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1 \
		GZ_GUI_PLUGIN_PATH="$GUI_PLUGIN_PATH" gz sim -g -v2
	printf '\n'
	start_process gui env GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1 \
		GZ_GUI_PLUGIN_PATH="$GUI_PLUGIN_PATH" gz sim -g -v2
fi

printf 'aircraft_id\tmodel\tmavsdk_udp\tgimbal_udp\tdyt_udp\trtsp_url\n' > "$RUNTIME_DIR/interfaces.tsv"
for ((i = 0; i < COUNT; i++)); do
	aircraft_id=$((i + 1))
	entity="rocket_two_axis_pod_${aircraft_id}"
	topic_prefix="/uav_${aircraft_id}/pod"
	mavsdk_port=$((14280 + i))
	udp_port=$((8770 + i))
	dyt_udp_port=$((9000 + i))
	rtsp_url="rtsp://${ADVERTISE_HOST}:${RTSP_PORT}/two_axis_pod_${aircraft_id}"
	rootfs="${RUNTIME_DIR}/rootfs_${aircraft_id}"
	mkdir -p "$rootfs"

	px4_command=(env GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1
		GZ_SIM_RESOURCE_PATH="$RESOURCE_PATH" GZ_SIM_SYSTEM_PLUGIN_PATH="$SYSTEM_PLUGIN_PATH"
		GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG_PATH"
		PX4_SIM_MODEL=gz_rocket_two_axis_pod PX4_GZ_STANDALONE=1
		PX4_GZ_WORLD="$WORLD" PX4_GZ_MODEL_NAME="$entity"
		"$PX4_BIN" -d "$RUNTIME_DIR/etc" -i "$i" -w "$rootfs")
	bridge_command=(env GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1
		GZ_SIM_RESOURCE_PATH="$RESOURCE_PATH" GZ_SIM_SYSTEM_PLUGIN_PATH="$SYSTEM_PLUGIN_PATH"
		GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG_PATH"
		GIMBAL_RUNDIR="${RUNTIME_DIR}/gimbal_${aircraft_id}"
		"$BRIDGE_PYTHON" -u "$BRIDGE_SERVICE/gimbal_bridge.py"
		--world "$WORLD" --gimbal-model "$entity"
		--topic-prefix "$topic_prefix" --image-topic "$topic_prefix/image"
		--udp-port "$udp_port" --rtsp-port "$RTSP_PORT"
		--mount /two_axis_pod --aircraft-id "$aircraft_id" --encoder "$ENCODER")

	echo
	echo "飞机 ${aircraft_id}："
	printf '  PX4 command: '
	printf '%q ' "${px4_command[@]}"
	printf '\n'
	printf '  吊仓网关 command: '
	printf '%q ' "${bridge_command[@]}"
	printf '\n'
	echo "  MAVSDK：        udpout://${ADVERTISE_HOST}:${mavsdk_port}"
	echo "  吊仓指令/遥测： udp://${ADVERTISE_HOST}:${udp_port}"
	echo "  Aisheng JSON：  udp://${ADVERTISE_HOST}:${dyt_udp_port}"
	echo "  吊仓视频：      ${rtsp_url}"
	printf '  地面站 command: '
	printf '%q ' bash \
		"$ASSETS_ROOT/models/gimbals/two_axis_pod/ground_station/run.sh" \
		--no-manage-bridge \
		--udp-host "$ADVERTISE_HOST" --udp-port "$udp_port" \
		--aircraft-id "$aircraft_id" --rtsp-port "$RTSP_PORT" \
		--mount /two_axis_pod --decoder nvdec --window-title "飞机${aircraft_id}吊仓"
	printf '\n'

	start_process "px4_${aircraft_id}" "${px4_command[@]}"
	wait_for_log "px4_${aircraft_id}" 'Startup script returned successfully' 60 || exit 1
	if ! "$DYT_UDP_BRIDGE_BIN" --instance "$i" start -p "$dyt_udp_port"; then
		echo "ERROR: 飞机 ${aircraft_id} 的 Aisheng UDP bridge 启动失败" >&2
		exit 1
	fi
	wait_for_log "px4_${aircraft_id}" "listening for Aisheng JSON on UDP 0.0.0.0:${dyt_udp_port}" 10 || exit 1
	sleep 0.2
	if ! "$DYT_UDP_BRIDGE_BIN" --instance "$i" status >/dev/null; then
		echo "ERROR: 飞机 ${aircraft_id} 的 Aisheng UDP bridge 启动后异常退出" >&2
		exit 1
	fi
	start_process "bridge_${aircraft_id}" "${bridge_command[@]}"
	wait_for_log "bridge_${aircraft_id}" '遥测/指令服务' 30 || exit 1
	wait_for_log "bridge_${aircraft_id}" 'RTSP 服务已启动' 60 || exit 1
	printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$aircraft_id" "$entity" "$mavsdk_port" "$udp_port" \
		"$dyt_udp_port" "$rtsp_url" \
		>> "$RUNTIME_DIR/interfaces.tsv"
done

collision_manager_command=(python3 -u "$SCRIPT_DIR/collision_manager.py"
	--aircraft-count "$COUNT" --px4-bin "$PX4_BIN"
	--metadata "$ASSETS_ROOT/worlds/$WORLD/metadata.json"
	--event-log "$RUNTIME_DIR/logs/collision-events.jsonl")
printf '碰撞致损服务 command: '
printf '%q ' "${collision_manager_command[@]}"
printf '\n'
start_process collision_manager "${collision_manager_command[@]}"
wait_for_log collision_manager '碰撞致损服务已就绪' 15 || exit 1

FAILED=0
trap - EXIT
echo
echo "场景已就绪：${COUNT} 架飞机，world=${WORLD}，资产版本=${ASSETS_REV}。"
echo "飞机保持待命，不自动解锁或飞行。"
echo "吊仓单目标跟踪：${TRACKING_BACKEND}。"
echo "碰撞致损：      PX4 飞机碰撞后关闭全部电机；事件记录 ${RUNTIME_DIR}/logs/collision-events.jsonl。"
if ((GUI)); then
	echo "雷达/靶机控制：使用 Gazebo 地图右侧的固定控制面板。"
	echo "地面站 JSON：   雷达航迹默认 1 Hz（可设 1～2 Hz），靶机真值 10 Hz；共享面板地址。"
	echo "QGC 靶机：      udp://${TARGET_QGC_ADDRESS}，SYSID=${TARGET_MAVLINK_SYSID}（只读轻量代理）。"
fi
echo "接口表：${RUNTIME_DIR}/interfaces.tsv"
echo "日志目录：${RUNTIME_DIR}/logs"
echo "停止场景：${SCRIPT_DIR}/stop.sh"

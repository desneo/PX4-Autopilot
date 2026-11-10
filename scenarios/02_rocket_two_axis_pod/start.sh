#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PX4_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
PX4_ETC="${PX4_DIR}/build/px4_sitl_default/etc"
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
ENABLE_COLLISION=0
AIRCRAFT_MODEL=rocket_two_axis_pod
FLIGHT_VIBRATION=off

usage() {
	cat <<'EOF'
用法：start.sh [-n 架数] [--gui] [--enable-collision] [--advertise-host 地址] [--encoder nvenc] [--tracking-backend sim_truth|csrt] [--aircraft-model 模型] [--flight-vibration off|afly5]

在 geo_32_217857_119_105878 world 中启动火箭穿越机与顶部吊仓/导引头组合模型。
默认启动 1 架、不开启 GUI，启动后等待 MAVSDK 和吊仓地面站控制。
靶机 command 服务默认启用，不自动生成靶机。

选项：
  -n, --count COUNT       飞机数量，1～10（默认：1）
      --gui               启动 Gazebo GUI
      --enable-collision  启用飞机之间的刚体碰撞和互相致损（默认关闭）
      --advertise-host IP 打印给远程客户端使用的本机地址
      --encoder TYPE      吊仓视频编码器：nvenc（默认：nvenc）
      --tracking-backend TYPE
                           单目标跟踪后端：sim_truth（默认）或 csrt；中科和A2416A仅支持 sim_truth
      --aircraft-model NAME
                           组合飞机模型：rocket_two_axis_pod（默认）、rocket_aisheng_seeker、
                           rocket_zhongke_d80 或 rocket_jielian_a2416a
      --flight-vibration PROFILE
                           悬停抖动配置：off（默认）或 afly5（实飞日志拟合）
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
		--enable-collision)
			ENABLE_COLLISION=1
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
		--aircraft-model)
			require_value "$@"
			AIRCRAFT_MODEL=$2
			shift 2
			;;
		--flight-vibration)
			require_value "$@"
			FLIGHT_VIBRATION=$2
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
if [[ $AIRCRAFT_MODEL != rocket_two_axis_pod && $AIRCRAFT_MODEL != rocket_aisheng_seeker \
	&& $AIRCRAFT_MODEL != rocket_zhongke_d80 && $AIRCRAFT_MODEL != rocket_jielian_a2416a ]]; then
	echo "ERROR: --aircraft-model 只接受 rocket_two_axis_pod、rocket_aisheng_seeker、rocket_zhongke_d80 或 rocket_jielian_a2416a" >&2
	exit 2
fi
if [[ ($AIRCRAFT_MODEL == rocket_zhongke_d80 || $AIRCRAFT_MODEL == rocket_jielian_a2416a) \
	&& $TRACKING_BACKEND != sim_truth ]]; then
	echo "ERROR: 中科D80和A2416A的 --tracking-backend 仅支持 sim_truth" >&2
	exit 2
fi
if [[ $FLIGHT_VIBRATION != off && $FLIGHT_VIBRATION != afly5 ]]; then
	echo "ERROR: --flight-vibration 只接受 off 或 afly5" >&2
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
	|| pgrep -f '[z]hongke-d80/controller/service/rtsp_relay[.]py' >/dev/null \
	|| pgrep -f '[a]isheng-ASD-2-075-DD0/gateway/(main|video_worker|rtsp_server)[.]py' >/dev/null \
	|| pgrep -f '[j]ielian-A2416A/(gateway/main|video/rtsp_server)[.]py' >/dev/null \
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
case "$AIRCRAFT_MODEL" in
	rocket_two_axis_pod)
		MODEL_DIR="assembledmodels/rocket_two_axis_pod"
		CONTROLLER_DIR="${ASSETS_ROOT}/models/gimbals/two_axis_pod/controller"
		BRIDGE_SERVICE="${CONTROLLER_DIR}/service"
		BRIDGE_KIND=two_axis
		PX4_UDP_BRIDGE=dyt_udp_bridge
		CONTROL_PORT_BASE=8770
		;;
	rocket_aisheng_seeker)
		MODEL_DIR="assembledmodels/rocket_aisheng_seeker"
		CONTROLLER_DIR="${ASSETS_ROOT}/models/gimbals/aisheng-ASD-2-075-DD0/controller"
		BRIDGE_SERVICE=""
		BRIDGE_KIND=aisheng
		PX4_UDP_BRIDGE=aisheng_udp_bridge
		CONTROL_PORT_BASE=8001
		;;
	rocket_zhongke_d80)
		MODEL_DIR="assembledmodels/rocket_zhongke_d80"
		CONTROLLER_DIR="${ASSETS_ROOT}/models/gimbals/zhongke-d80/controller"
		BRIDGE_SERVICE="${CONTROLLER_DIR}/service"
		BRIDGE_KIND=zhongke
		PX4_UDP_BRIDGE=zhongke_udp_bridge
		CONTROL_PORT_BASE=8880
		if [[ -z ${GIMBAL_PYTHON:-} && -x "$ASSETS_ROOT/models/gimbals/.venv-zhongke-d80/bin/python" ]]; then
			BRIDGE_PYTHON="$ASSETS_ROOT/models/gimbals/.venv-zhongke-d80/bin/python"
		fi
		;;
	rocket_jielian_a2416a)
		MODEL_DIR="assembledmodels/rocket_jielian_a2416a"
		CONTROLLER_DIR="${ASSETS_ROOT}/models/gimbals/jielian-A2416A"
		BRIDGE_SERVICE=""
		BRIDGE_KIND=jielian
		PX4_UDP_BRIDGE=aisheng_udp_bridge
		CONTROL_PORT_BASE=8001
		if [[ -z ${GIMBAL_PYTHON:-} && -x "$ASSETS_ROOT/.venv-linux/bin/python" ]]; then
			BRIDGE_PYTHON="$ASSETS_ROOT/.venv-linux/bin/python"
		fi
		;;
esac
PX4_UDP_BRIDGE_BIN=""
[[ -n $PX4_UDP_BRIDGE ]] && PX4_UDP_BRIDGE_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4-${PX4_UDP_BRIDGE}"
GENERATOR="${ASSETS_ROOT}/${MODEL_DIR}/tools/generate_dynamic_model.py"
required=("$WORLD_SOURCE" "$GENERATOR" "$CONTROLLER_DIR/CMakeLists.txt" \
	"$ASSETS_ROOT/worlds/${WORLD}/metadata.json" \
	"$ASSETS_ROOT/models/vehicles/xiaomotuo/model.sdf" \
	"$ASSETS_ROOT/models/vehicles/xiaobianfu/model.sdf" \
	"$SCRIPT_DIR/collision_manager.py" \
	"$SCRIPT_DIR/target_manager.py" \
	"$SCRIPT_DIR/radar_manager.py")
if [[ $BRIDGE_KIND == two_axis || $BRIDGE_KIND == zhongke ]]; then
	required+=("$BRIDGE_SERVICE/gimbal_bridge.py")
	if [[ $BRIDGE_KIND == zhongke ]]; then
		required+=("$ASSETS_ROOT/models/gimbals/zhongke-d80/parameters.ini"
			"$ASSETS_ROOT/models/gimbals/zhongke-d80/ground_station/ground_station.py"
			"$BRIDGE_SERVICE/rtsp_relay.py")
	fi
elif [[ $BRIDGE_KIND == aisheng ]]; then
	required+=(
		"$ASSETS_ROOT/models/gimbals/aisheng-ASD-2-075-DD0/gateway/main.py"
		"$ASSETS_ROOT/models/gimbals/aisheng-ASD-2-075-DD0/ground_station/main.py"
		"$ASSETS_ROOT/models/gimbals/aisheng-ASD-2-075-DD0/common/protocol.py"
	)
else
	required+=(
		"$ASSETS_ROOT/models/gimbals/jielian-A2416A/gateway/main.py"
		"$ASSETS_ROOT/models/gimbals/jielian-A2416A/video/rtsp_server.py"
		"$ASSETS_ROOT/models/gimbals/jielian-A2416A/ground_station/run.sh"
		"$ASSETS_ROOT/models/gimbals/jielian-A2416A/ground_station/config.ini"
		"$ASSETS_ROOT/models/gimbals/jielian-A2416A/video/hardware.py"
	)
fi
VIBRATION_MODEL_DIR="${ASSETS_ROOT}/models/vehicles/rocket_racing_quad"
if [[ $FLIGHT_VIBRATION != off ]]; then
	required+=("$VIBRATION_MODEL_DIR/controller/CMakeLists.txt"
		"$VIBRATION_MODEL_DIR/tools/configure_vibration.py"
		"$VIBRATION_MODEL_DIR/config/flight_hover.json")
fi
for required_file in "${required[@]}"; do
	[[ -f $required_file ]] || { echo "ERROR: 最新资产仓缺少必要文件：${required_file}" >&2; exit 1; }
done

for command in gz setsid timeout git cmake make find; do
	command -v "$command" >/dev/null || { echo "ERROR: 缺少命令：${command}" >&2; exit 1; }
done

if [[ ! -x $PX4_BIN || ! -d $PX4_ETC ]] \
	|| [[ -n $PX4_UDP_BRIDGE_BIN && ! -x $PX4_UDP_BRIDGE_BIN ]]; then
	echo "PX4 SITL 尚未构建，开始构建 px4_sitl_default。"
	make -C "$PX4_DIR" px4_sitl_default
fi

if [[ $BRIDGE_KIND == zhongke ]]; then
	echo "增量构建中科D80控制插件。"
elif [[ $BRIDGE_KIND == two_axis ]]; then
	echo "增量构建两轴吊仓控制插件。"
elif [[ $BRIDGE_KIND == jielian ]]; then
	echo "增量构建A2416A捷联导引头控制插件。"
else
	echo "增量构建艾晟导引头控制插件。"
fi
# 场景启动只构建运行插件，不要求资产仓提供可选测试源码。
cmake -S "$CONTROLLER_DIR" -B "$CONTROLLER_DIR/build" \
	-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build "$CONTROLLER_DIR/build" -j"$(nproc)"

if [[ $FLIGHT_VIBRATION != off ]]; then
	echo "增量构建火箭机悬停抖动插件（${FLIGHT_VIBRATION}）。"
	cmake -S "$VIBRATION_MODEL_DIR/controller" -B "$VIBRATION_MODEL_DIR/controller/build" \
		-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
	cmake --build "$VIBRATION_MODEL_DIR/controller/build" -j"$(nproc)"
fi

GUI_PLUGIN_DIR="${ASSETS_ROOT}/worlds/${WORLD}/gui"
echo "增量构建 02 场景碰撞监控与靶机插件。"
if ((GUI)); then
	cmake -S "$SCENARIO_GUI_DIR" -B "$SCENARIO_GUI_DIR/build" \
		-DCMAKE_BUILD_TYPE=Release -DSCENARIO02_BUILD_GUI=ON -DBUILD_TESTING=OFF
	cmake --build "$SCENARIO_GUI_DIR/build" -j"$(nproc)"
else
	cmake -S "$SCENARIO_GUI_DIR" -B "$SCENARIO_GUI_DIR/build" \
		-DCMAKE_BUILD_TYPE=Release -DSCENARIO02_BUILD_GUI=OFF -DBUILD_TESTING=OFF
	cmake --build "$SCENARIO_GUI_DIR/build" --target CollisionMonitor TargetTrajectory -j"$(nproc)"
fi
if ((GUI)); then
	echo "增量构建 world 的 Gazebo GUI 插件。"
	cmake -S "$GUI_PLUGIN_DIR" -B "$GUI_PLUGIN_DIR/build" -DCMAKE_BUILD_TYPE=Release
	cmake --build "$GUI_PLUGIN_DIR/build" -j"$(nproc)"
fi

python3 -c 'import numpy, gz.transport13, gz.msgs10.stringmsg_pb2, gz.msgs10.pose_v_pb2'
"$BRIDGE_PYTHON" -c 'import gi, gz.transport13; gi.require_version("Gst", "1.0"); from gi.repository import Gst'
if [[ $BRIDGE_KIND == zhongke ]]; then
	"$BRIDGE_PYTHON" - <<'PY_ZHONGKE_DEPS'
import cv2, numpy, gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer
Gst.init(None)
required = ['appsrc', 'videoconvert', 'nvcudah264enc', 'nvh264dec',
            'rtph264pay', 'rtph264depay', 'udpsrc', 'udpsink']
missing = [name for name in required if Gst.ElementFactory.find(name) is None]
if missing:
    raise SystemExit('ERROR: 缺少中科视频插件：' + ','.join(missing))
PY_ZHONGKE_DEPS
elif [[ $BRIDGE_KIND == jielian ]]; then
	PYTHONPATH="$CONTROLLER_DIR" "$BRIDGE_PYTHON" - <<'PY_JIELIAN_DEPS'
import cv2, gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer
from video.hardware import require_nvcodec
Gst.init(None)
require_nvcodec(need_encoder=True, need_decoder=True)
missing = [name for name in ('appsrc', 'videoconvert', 'rtph264pay',
                              'rtph264depay', 'h264parse', 'shmsink', 'shmsrc',
                              'udpsrc', 'udpsink')
           if Gst.ElementFactory.find(name) is None]
if missing:
    raise SystemExit('ERROR: 缺少A2416A视频插件：' + ','.join(missing))
PY_JIELIAN_DEPS
fi

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

python3 - "$COUNT" "$BRIDGE_KIND" <<'PY_PORTS'
import socket
import sys

count = int(sys.argv[1])
kind = sys.argv[2]
ranges = [(dict(aisheng=8001, two_axis=8770, zhongke=8880, jielian=8001)[kind], count)]
if kind == "aisheng":
    ranges += [(9000, count), (7001, count), (8554, count)]
if kind == "jielian":
    ranges += [(9000, count), (7001, count), (14280, count),
               (18570, count), (14580, count), (13030, count)]
if kind == "zhongke":
    ranges += [(9000, count), (14280, count), (18570, count), (14580, count), (13030, count)]
probes = [(socket.SOCK_DGRAM, base + offset) for base, length in ranges for offset in range(length)]
if kind == "zhongke":
    probes += [(socket.SOCK_STREAM, 8566 + offset) for offset in range(count)]
if kind == "jielian":
    probes += [(socket.SOCK_STREAM, 8554 + offset) for offset in range(count)]
sockets = []
try:
    for kind, port in probes:
        sock = socket.socket(socket.AF_INET, kind)
        sockets.append(sock)
        try:
            sock.bind(("0.0.0.0", port))
        except OSError as error:
            protocol = "UDP" if kind == socket.SOCK_DGRAM else "RTSP TCP"
            raise SystemExit(f"ERROR: 吊仓 {protocol} 端口 {port} 已被使用：{error}")
finally:
    for sock in sockets:
        sock.close()
PY_PORTS

rm -rf -- "$RUNTIME_DIR"
mkdir -p -- "$RUNTIME_DIR/models" "$RUNTIME_DIR/logs" "$RUNTIME_DIR/pids" \
	"$RUNTIME_DIR/target" "$RUNTIME_DIR/ground_station"
printf '%s\n' "$COUNT" > "$RUNTIME_DIR/count"
printf '%s\n' "$ASSETS_ROOT" > "$RUNTIME_DIR/assets_root"
printf '%s\n' "$ASSETS_REV" > "$RUNTIME_DIR/assets_revision"

# 同一组运行字段分别传给生成器和网关，确保中科有效配置及hash一致。
zhongke_targets=()
for ((aircraft_id = 1; aircraft_id <= COUNT; aircraft_id++)); do
	zhongke_targets+=(--target-name "${AIRCRAFT_MODEL}_${aircraft_id}")
done
zhongke_targets+=(--target-name xiaomotuo --target-name xiaobianfu)
zhongke_runtime_args() {
	local aircraft_id=$1
	local index=$((aircraft_id - 1))
	zhongke_args=(--config "$ASSETS_ROOT/models/gimbals/zhongke-d80/parameters.ini"
		--model "${AIRCRAFT_MODEL}_${aircraft_id}" --aircraft-id "$aircraft_id"
		--world "$WORLD" "${zhongke_targets[@]}" --udp-bind 0.0.0.0
		--udp-port "$((CONTROL_PORT_BASE + index))" --rtsp-bind 0.0.0.0
		--rtsp-port "$((8566 + index))" --rtsp-mount "/zhongke_d80_${aircraft_id}")
}

model_args=()
for ((i = 0; i < COUNT; i++)); do
	aircraft_id=$((i + 1))
	generated="${RUNTIME_DIR}/generated/aircraft_${aircraft_id}/${AIRCRAFT_MODEL}"
	generator_command=(python3 "$GENERATOR" --output-dir "$generated" --topic-prefix "/uav_${aircraft_id}/pod")
	if [[ $BRIDGE_KIND == zhongke ]]; then
		zhongke_runtime_args "$aircraft_id"
		generator_command=("$BRIDGE_PYTHON" "$GENERATOR" --output-dir "$generated"
			--topic-prefix "/uav_${aircraft_id}/pod" "${zhongke_args[@]}")
	fi
	"${generator_command[@]}"
	model_args+=(--model "$aircraft_id" "$generated/model.sdf")
done

WORLD_FILE="${RUNTIME_DIR}/world.sdf"
prepare_world_command=(python3 "$SCRIPT_DIR/prepare_world.py"
	--source-world "$WORLD_SOURCE" --output "$WORLD_FILE"
	--model-basename "$AIRCRAFT_MODEL"
	--tracking-backend "$TRACKING_BACKEND" "${model_args[@]}")
((GUI)) && prepare_world_command+=(--target-control)
((ENABLE_COLLISION)) && prepare_world_command+=(--enable-collision)
if [[ $FLIGHT_VIBRATION != off ]]; then
	prepare_world_command+=(--flight-vibration "$FLIGHT_VIBRATION")
fi
"${prepare_world_command[@]}"
gz sdf -k "$WORLD_FILE"

cp -R "$PX4_ETC" "$RUNTIME_DIR/etc"
rm -f "$RUNTIME_DIR/etc/init.d-posix/airframes/"*_gz_"$AIRCRAFT_MODEL"
mapfile -t generated_airframes < <(find \
	"$RUNTIME_DIR/generated/aircraft_1/${AIRCRAFT_MODEL}/px4" -maxdepth 1 -type f \
	-name "*_gz_${AIRCRAFT_MODEL}" -print)
if ((${#generated_airframes[@]} != 1)); then
	echo "ERROR: 最新资产仓应生成一个 ${AIRCRAFT_MODEL} PX4 机体文件，实际为 ${#generated_airframes[@]} 个。" >&2
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
if [[ $FLIGHT_VIBRATION != off ]]; then
	python3 "$VIBRATION_MODEL_DIR/tools/configure_vibration.py" \
		--airframe "$runtime_airframe" --profile "$VIBRATION_MODEL_DIR/config/flight_hover.json"
fi

TARGET_SYSTEM_PLUGIN_DIR="${SCENARIO_GUI_DIR}/build/target-system-plugins"

RESOURCE_PATH="${RUNTIME_DIR}/models:${ASSETS_ROOT}/assembledmodels:${ASSETS_ROOT}/models/vehicles:${ASSETS_ROOT}/models/gimbals${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
SYSTEM_PLUGIN_PATH="${CONTROLLER_DIR}/build/lib"
SYSTEM_PLUGIN_PATH="${SYSTEM_PLUGIN_PATH}:${TARGET_SYSTEM_PLUGIN_DIR}"
if [[ $FLIGHT_VIBRATION != off ]]; then
	SYSTEM_PLUGIN_PATH="${SYSTEM_PLUGIN_PATH}:${VIBRATION_MODEL_DIR}/controller/build/lib"
fi
SYSTEM_PLUGIN_PATH="${SYSTEM_PLUGIN_PATH}${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
SERVER_CONFIG_PATH="${ASSETS_ROOT}/models/gimbals/two_axis_pod/scripts/server.config"
if [[ $BRIDGE_KIND == zhongke ]]; then
	# 世界已声明物理与传感器系统，避免加载两轴吊仓专用配置。
	SERVER_CONFIG_PATH="${RUNTIME_DIR}/server.config"
	printf '<server_config><plugins/></server_config>\n' > "$SERVER_CONFIG_PATH"
fi
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
		if ! kill -0 "$pid" 2>/dev/null; then
			echo "ERROR: ${name} 启动期间退出" >&2
			tail -n 50 "$log_file" >&2 || true
			return 1
		fi
		grep -q -- "$pattern" "$log_file" 2>/dev/null && return 0
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
	entity="${AIRCRAFT_MODEL}_${aircraft_id}"
	imu_topic="/world/${WORLD}/model/${entity}/link/base_link/sensor/imu_sensor/imu"
	if ! wait_for_gz_message "imu_${aircraft_id}" "$imu_topic" 120; then
		echo "ERROR: 飞机 ${aircraft_id} 的 IMU 未就绪：${imu_topic}" >&2
		tail -n 30 "${RUNTIME_DIR}/logs/gazebo.log" >&2 || true
		exit 1
	fi
done

target_manager_command=(python3 -u "$SCRIPT_DIR/target_manager.py"
	--world "$WORLD" --runtime-dir "$RUNTIME_DIR" --assets-root "$ASSETS_ROOT"
	--qgc-address "$TARGET_QGC_ADDRESS"
	--mavlink-system-id "$TARGET_MAVLINK_SYSID")
printf '靶机管理服务 command: '
printf '%q ' "${target_manager_command[@]}"
printf '\n'
start_process target_manager "${target_manager_command[@]}"
wait_for_log target_manager '靶机管理服务已就绪' 15 || exit 1

if ((GUI)); then
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

if [[ $BRIDGE_KIND == two_axis ]]; then
	printf 'aircraft_id\tmodel\tmavsdk_udp\tgimbal_udp\tdyt_udp\trtsp_url\n' > "$RUNTIME_DIR/interfaces.tsv"
elif [[ $BRIDGE_KIND == zhongke ]]; then
	printf 'aircraft_id\tmodel\tmavsdk_udp\tcontrol_udp\tzhongke_udp\trtsp_url\n' > "$RUNTIME_DIR/interfaces.tsv"
elif [[ $BRIDGE_KIND == jielian ]]; then
	printf 'aircraft_id\tmodel\tmavsdk_udp\tcontrol_udp\taisheng_udp\trtsp_url\tground_station_config\n' > "$RUNTIME_DIR/interfaces.tsv"
else
	printf 'aircraft_id\tmodel\tmavsdk_udp\tcontrol_udp\taisheng_udp\trtsp_url\n' > "$RUNTIME_DIR/interfaces.tsv"
fi
for ((i = 0; i < COUNT; i++)); do
	aircraft_id=$((i + 1))
	entity="${AIRCRAFT_MODEL}_${aircraft_id}"
	topic_prefix="/uav_${aircraft_id}/pod"
	mavsdk_port=$((14280 + i))
	udp_port=$((CONTROL_PORT_BASE + i))
	dyt_udp_port=$((9000 + i))
	rootfs="${RUNTIME_DIR}/rootfs_${aircraft_id}"
	mkdir -p "$rootfs"
	if [[ $BRIDGE_KIND == two_axis ]]; then
		rtsp_url="rtsp://${ADVERTISE_HOST}:${RTSP_PORT}/two_axis_pod_${aircraft_id}"
	elif [[ $BRIDGE_KIND == zhongke ]]; then
		zhongke_runtime_args "$aircraft_id"
		rtsp_port=$((8566 + i))
		rtsp_url="rtsp://${ADVERTISE_HOST}:${rtsp_port}/zhongke_d80_${aircraft_id}"
	elif [[ $BRIDGE_KIND == aisheng ]]; then
		video_port=$((7001 + i))
		rtsp_port=$((8554 + i))
		rtsp_url="rtsp://${ADVERTISE_HOST}:${rtsp_port}/aisheng_seeker"
	else
		video_port=$((7001 + i))
		rtsp_port=$((8554 + i))
		rtsp_mount=/jielian_a2416a
		rtsp_url="rtsp://${ADVERTISE_HOST}:${rtsp_port}${rtsp_mount}"
		ground_station_config="${RUNTIME_DIR}/ground_station/aircraft_${aircraft_id}.ini"
		{
			printf '[network]\n'
			printf '# 导引头网关的控制/遥测共用端口。\n'
			printf 'host = %s\nport = %d\n\n' "$ADVERTISE_HOST" "$udp_port"
			printf '[forward]\n'
			printf '# “发送艾晟报文”将每个100 Hz有效遥测帧发送到本机飞控监听端口。\n'
			printf 'ip = %s\nport = %d\n\n' "$ADVERTISE_HOST" "$dyt_udp_port"
			printf '[video]\n'
			printf '# A2416A H.264 RTSP流。\n'
			printf 'rtsp_url = %s\nwidth = 1920\nheight = 1080\nfps = 25\n\n' "$rtsp_url"
			printf '[display]\n'
			printf 'width = 960\nheight = 540\nwindow_title = A2416A 捷联导引头地面站 · 飞机 %d\n' "$aircraft_id"
		} > "$ground_station_config"
	fi

	px4_command=(env GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1
		GZ_SIM_RESOURCE_PATH="$RESOURCE_PATH" GZ_SIM_SYSTEM_PLUGIN_PATH="$SYSTEM_PLUGIN_PATH"
		GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG_PATH"
		PX4_SIM_MODEL="gz_${AIRCRAFT_MODEL}" PX4_GZ_STANDALONE=1
		PX4_GZ_WORLD="$WORLD" PX4_GZ_MODEL_NAME="$entity"
		"$PX4_BIN" -d "$RUNTIME_DIR/etc" -i "$i" -w "$rootfs")
	if [[ $BRIDGE_KIND == two_axis ]]; then
		bridge_command=(env GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1
			GZ_SIM_RESOURCE_PATH="$RESOURCE_PATH" GZ_SIM_SYSTEM_PLUGIN_PATH="$SYSTEM_PLUGIN_PATH"
			GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG_PATH"
			GIMBAL_RUNDIR="${RUNTIME_DIR}/gimbal_${aircraft_id}"
			"$BRIDGE_PYTHON" -u "$BRIDGE_SERVICE/gimbal_bridge.py"
			--world "$WORLD" --gimbal-model "$entity"
			--topic-prefix "$topic_prefix" --image-topic "$topic_prefix/image"
			--udp-port "$udp_port" --rtsp-port "$RTSP_PORT"
			--mount /two_axis_pod --aircraft-id "$aircraft_id" --encoder "$ENCODER")
	elif [[ $BRIDGE_KIND == zhongke ]]; then
		bridge_command=(env GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1
			GZ_SIM_RESOURCE_PATH="$RESOURCE_PATH" GZ_SIM_SYSTEM_PLUGIN_PATH="$SYSTEM_PLUGIN_PATH"
			GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG_PATH"
			"$BRIDGE_PYTHON" -u "$BRIDGE_SERVICE/gimbal_bridge.py"
			--topic-prefix "$topic_prefix" --image-topic "$topic_prefix/image"
			"${zhongke_args[@]}")
	elif [[ $BRIDGE_KIND == aisheng ]]; then
		bridge_command=(env GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1
			GZ_SIM_RESOURCE_PATH="$RESOURCE_PATH" GZ_SIM_SYSTEM_PLUGIN_PATH="$SYSTEM_PLUGIN_PATH"
			GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG_PATH"
			"$BRIDGE_PYTHON" -u \
			"$ASSETS_ROOT/models/gimbals/aisheng-ASD-2-075-DD0/gateway/main.py"
			--control-host 0.0.0.0 --control-port "$udp_port"
			--topic-prefix "$topic_prefix"
			--image-topic "$topic_prefix/image"
			--tele-image-topic "$topic_prefix/image_tele"
			--video --video-port "$video_port"
			--rtsp-port "$rtsp_port" --rtsp-mount /aisheng_seeker
			--verbose)
	else
		bridge_command=(env GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1
			GZ_SIM_RESOURCE_PATH="$RESOURCE_PATH" GZ_SIM_SYSTEM_PLUGIN_PATH="$SYSTEM_PLUGIN_PATH"
			GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG_PATH"
			PYTHONPATH="$CONTROLLER_DIR"
			"$BRIDGE_PYTHON" -u "$CONTROLLER_DIR/gateway/main.py"
			--host 0.0.0.0 --port "$udp_port"
			--topic-prefix "$topic_prefix" --image-topic "$topic_prefix/image"
			--rtp-host 127.0.0.1 --rtp-port "$video_port" --verbose)
		rtsp_command=(env PYTHONPATH="$CONTROLLER_DIR"
			"$BRIDGE_PYTHON" -u "$CONTROLLER_DIR/video/rtsp_server.py"
			--rtp-port "$video_port" --rtsp-port "$rtsp_port" --mount "$rtsp_mount")
	fi

	echo
	echo "飞机 ${aircraft_id}："
	printf '  PX4 command: '
	printf '%q ' "${px4_command[@]}"
	printf '\n'
	if [[ $BRIDGE_KIND == zhongke ]]; then
		printf '  中科网关 command: '
	elif [[ $BRIDGE_KIND == two_axis ]]; then
		printf '  吊仓网关 command: '
	else
		printf '  导引头网关 command: '
	fi
	printf '%q ' "${bridge_command[@]}"
	printf '\n'
	if [[ $BRIDGE_KIND == jielian ]]; then
		printf '  A2416A RTSP command: '
		printf '%q ' "${rtsp_command[@]}"
		printf '\n'
	fi
	echo "  MAVSDK：        udpout://${ADVERTISE_HOST}:${mavsdk_port}"
	if [[ $BRIDGE_KIND == zhongke ]]; then
		echo "  中科指令/遥测： udp://${ADVERTISE_HOST}:${udp_port}（16字节控制/32字节遥测）"
	elif [[ $BRIDGE_KIND == two_axis ]]; then
		echo "  吊仓指令/遥测： udp://${ADVERTISE_HOST}:${udp_port}"
	else
		echo "  导引头指令/遥测：udp://${ADVERTISE_HOST}:${udp_port}"
	fi
	if [[ $BRIDGE_KIND == zhongke ]]; then
		echo "  中科 JSON：     udp://${ADVERTISE_HOST}:${dyt_udp_port}"
		echo "  地面站转发：    将目标设为 ${ADVERTISE_HOST}:${dyt_udp_port}，手动点击“发送中科报文”开启。"
	elif [[ -n $PX4_UDP_BRIDGE ]]; then
		echo "  Aisheng JSON：  udp://${ADVERTISE_HOST}:${dyt_udp_port}"
		if [[ $BRIDGE_KIND == jielian ]]; then
			echo "  地面站转发：    点击“发送艾晟报文”后发送到飞控监听端口。"
		fi
	fi
	if [[ $BRIDGE_KIND == two_axis ]]; then
		echo "  吊仓视频：      ${rtsp_url}"
		printf '  地面站 command: '
		printf '%q ' bash \
			"$ASSETS_ROOT/models/gimbals/two_axis_pod/ground_station/run.sh" \
			--no-manage-bridge \
			--udp-host "$ADVERTISE_HOST" --udp-port "$udp_port" \
			--aircraft-id "$aircraft_id" --rtsp-port "$RTSP_PORT" \
			--mount /two_axis_pod --decoder nvdec --window-title "Aircraft${aircraft_id}Gimbal"
	elif [[ $BRIDGE_KIND == zhongke ]]; then
		echo "  中科视频：      ${rtsp_url}"
		printf '  地面站 command: '
		printf '%q ' "$BRIDGE_PYTHON" \
			"$ASSETS_ROOT/models/gimbals/zhongke-d80/ground_station/ground_station.py" \
			--config "$ASSETS_ROOT/models/gimbals/zhongke-d80/parameters.ini" \
			--udp-host "$ADVERTISE_HOST" --udp-port "$udp_port" \
			--rtsp-url "$rtsp_url" --window-title "中科 D80 · 飞机 ${aircraft_id}"
	elif [[ $BRIDGE_KIND == aisheng ]]; then
		echo "  导引头视频：    ${rtsp_url}（广角/长焦共用）"
		printf '  地面站 command: '
		printf '%q ' "$BRIDGE_PYTHON" \
			"$ASSETS_ROOT/models/gimbals/aisheng-ASD-2-075-DD0/ground_station/main.py" \
			--host "$ADVERTISE_HOST" --control-port "$udp_port" \
			--forward-host "$ADVERTISE_HOST" --forward-port "$dyt_udp_port" \
			--rtsp-url "$rtsp_url"
	else
		echo "  A2416A视频：    ${rtsp_url}"
		echo "  地面站配置：    ${ground_station_config}"
		printf '  地面站 command: '
		printf '%q ' bash "$CONTROLLER_DIR/ground_station/run.sh" \
			--config "$ground_station_config"
	fi
	printf '\n'

	start_process "px4_${aircraft_id}" "${px4_command[@]}"
	wait_for_log "px4_${aircraft_id}" 'Startup script returned successfully' 60 || exit 1
	if [[ -n $PX4_UDP_BRIDGE_BIN ]]; then
		if ! "$PX4_UDP_BRIDGE_BIN" --instance "$i" start -p "$dyt_udp_port"; then
			echo "ERROR: 飞机 ${aircraft_id} 的 ${PX4_UDP_BRIDGE} 启动失败" >&2
			exit 1
		fi
		if [[ $BRIDGE_KIND == two_axis ]]; then
			bridge_ready="listening for Aisheng JSON on UDP 0.0.0.0:${dyt_udp_port}"
		elif [[ $BRIDGE_KIND == zhongke ]]; then
			bridge_ready="中科遥测接收已就绪 UDP 0.0.0.0:${dyt_udp_port}"
		else
			bridge_ready="完整遥测接收已就绪 UDP 0.0.0.0:${dyt_udp_port}"
		fi
		wait_for_log "px4_${aircraft_id}" "$bridge_ready" 10 || exit 1
		sleep 0.2
		if ! "$PX4_UDP_BRIDGE_BIN" --instance "$i" status >/dev/null; then
			echo "ERROR: 飞机 ${aircraft_id} 的 ${PX4_UDP_BRIDGE} 启动后异常退出" >&2
			exit 1
		fi
	fi
	if [[ $BRIDGE_KIND == jielian ]]; then
		start_process "rtsp_${aircraft_id}" "${rtsp_command[@]}"
	fi
	start_process "bridge_${aircraft_id}" "${bridge_command[@]}"
	if [[ $BRIDGE_KIND == jielian ]]; then
		# RTSP服务会在启动时预备udpsrc并等待首帧；网关必须并行启动，
		# 不能在启动网关之前阻塞等待RTSP就绪日志。
		wait_for_log "rtsp_${aircraft_id}" 'A2416A RTSP服务：' 30 || exit 1
	fi
	if [[ $BRIDGE_KIND == two_axis ]]; then
		wait_for_log "bridge_${aircraft_id}" '遥测/指令服务' 30 || exit 1
		wait_for_log "bridge_${aircraft_id}" 'RTSP 服务已启动' 60 || exit 1
		printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$aircraft_id" "$entity" "$mavsdk_port" "$udp_port" \
			"$dyt_udp_port" "$rtsp_url" \
			>> "$RUNTIME_DIR/interfaces.tsv"
	elif [[ $BRIDGE_KIND == zhongke ]]; then
		wait_for_log "bridge_${aircraft_id}" '中科网关启动' 30 || exit 1
		wait_for_log "bridge_${aircraft_id}" '中科RTSP已启动' 60 || exit 1
		printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$aircraft_id" "$entity" "$mavsdk_port" "$udp_port" \
			"$dyt_udp_port" "$rtsp_url" \
			>> "$RUNTIME_DIR/interfaces.tsv"
	elif [[ $BRIDGE_KIND == jielian ]]; then
		wait_for_log "bridge_${aircraft_id}" 'A2416A网关：' 30 || exit 1
		wait_for_log "bridge_${aircraft_id}" 'A2416A H.264视频：' 30 || exit 1
		printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$aircraft_id" "$entity" \
			"$mavsdk_port" "$udp_port" "$dyt_udp_port" "$rtsp_url" \
			"$ground_station_config" >> "$RUNTIME_DIR/interfaces.tsv"
	else
		wait_for_log "bridge_${aircraft_id}" '网关启动' 30 || exit 1
		wait_for_log "bridge_${aircraft_id}" 'RTSP 服务已启动' 60 || exit 1
		printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$aircraft_id" "$entity" \
			"$mavsdk_port" "$udp_port" "$dyt_udp_port" "$rtsp_url" \
			>> "$RUNTIME_DIR/interfaces.tsv"
	fi
done

collision_manager_command=(python3 -u "$SCRIPT_DIR/collision_manager.py"
	--aircraft-count "$COUNT" --px4-bin "$PX4_BIN"
	--metadata "$ASSETS_ROOT/worlds/$WORLD/metadata.json"
	--event-log "$RUNTIME_DIR/logs/collision-events.jsonl")
((ENABLE_COLLISION)) && collision_manager_command+=(--enable-collision)
printf '碰撞致损服务 command: '
printf '%q ' "${collision_manager_command[@]}"
printf '\n'
start_process collision_manager "${collision_manager_command[@]}"
wait_for_log collision_manager '碰撞致损服务已就绪' 15 || exit 1

FAILED=0
trap - EXIT
echo
if [[ $BRIDGE_KIND == two_axis ]]; then
	echo "场景已就绪：${COUNT} 架飞机，world=${WORLD}，资产版本=${ASSETS_REV}。"
else
	echo "场景已就绪：${COUNT} 架飞机，model=${AIRCRAFT_MODEL}，world=${WORLD}，资产版本=${ASSETS_REV}。"
fi
echo "飞机保持待命，不自动解锁或飞行。"
if [[ $FLIGHT_VIBRATION != off ]]; then
	echo "悬停抖动：      ${FLIGHT_VIBRATION}，已启用运行副本扰动；配置摘要 ${RUNTIME_DIR}/flight_vibration.json。"
fi
if [[ $BRIDGE_KIND == two_axis ]]; then
	echo "吊仓单目标跟踪：${TRACKING_BACKEND}。"
else
	echo "导引头单目标跟踪：${TRACKING_BACKEND}。"
fi
if ((ENABLE_COLLISION)); then
	echo "机间碰撞：      已启用刚体接触；相撞飞机关闭全部电机。"
else
	echo "机间碰撞：      已关闭刚体接触和互相致损；可用 --enable-collision 启用。"
fi
echo "其他碰撞：      飞机与地形/建筑物/靶机的检测和致损保持启用；事件记录 ${RUNTIME_DIR}/logs/collision-events.jsonl。"
echo "靶机 command： /scenario/02/target/command（默认无靶机，等待启动命令）。"
echo "QGC 靶机：      udp://${TARGET_QGC_ADDRESS}，SYSID=${TARGET_MAVLINK_SYSID}（只读轻量代理）。"
if ((GUI)); then
	echo "雷达/靶机控制：使用 Gazebo 地图右侧的固定控制面板。"
	echo "地面站 JSON：   雷达航迹默认 1 Hz（可设 1～2 Hz）。"
fi
echo "接口表：${RUNTIME_DIR}/interfaces.tsv"
echo "日志目录：${RUNTIME_DIR}/logs"
echo "停止场景：${SCRIPT_DIR}/stop.sh"

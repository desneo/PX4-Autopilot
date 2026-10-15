#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PX4_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
RUNTIME_DIR="${TMPDIR:-/tmp}/px4-scenarios/02_rocket_two_axis_pod"
DEFAULT_MAVSDK_PREFIX=$(cd -- "${PX4_DIR}/.." && pwd)/MAVSDK/install
COUNT=
SITL_HOST=${SITL_HOST:-}
MISSION_BIN=
MAVSDK_PREFIX=${MAVSDK_PREFIX:-${DEFAULT_MAVSDK_PREFIX}}
MAVSDK_LIB_DIR=

usage() {
	cat <<'EOF'
用法：test_mavsdk_route.sh [-n 架数] [--sitl-host 地址]
                            [--mission-bin 文件] [--mavsdk-prefix 目录]
                            [--mavsdk-lib-dir 目录]

通过 udpout://主机:14280+i 依次控制场景中的飞机完成航线、返航并落地。
未指定 -n 时，从当前场景 interfaces.tsv 自动读取架数。

选项：
  -n, --count 架数       飞机数量，1～10
      --sitl-host 地址   PX4 SITL 所在机器的 IPv4 地址（必填）
      --mission-bin 文件 使用已有测试程序，不执行本地编译
      --mavsdk-prefix 目录
                         预编译 MAVSDK 安装前缀，默认 ../MAVSDK/install
      --mavsdk-lib-dir 目录
                         测试程序运行时使用的 libmavsdk.so 目录
  -h, --help             显示帮助
EOF
}

while (($# > 0)); do
	case "$1" in
		-n|--count)
			(($# >= 2)) || { echo "ERROR: $1 需要参数" >&2; exit 2; }
			COUNT=$2
			shift 2
			;;
		--sitl-host)
			(($# >= 2)) || { echo "ERROR: $1 需要参数" >&2; exit 2; }
			SITL_HOST=$2
			shift 2
			;;
		--mission-bin)
			(($# >= 2)) || { echo "ERROR: $1 需要参数" >&2; exit 2; }
			MISSION_BIN=$(realpath "$2")
			shift 2
			;;
		--mavsdk-prefix)
			(($# >= 2)) || { echo "ERROR: $1 需要参数" >&2; exit 2; }
			MAVSDK_PREFIX=$(realpath "$2")
			shift 2
			;;
		--mavsdk-lib-dir)
			(($# >= 2)) || { echo "ERROR: $1 需要参数" >&2; exit 2; }
			MAVSDK_LIB_DIR=$(realpath "$2")
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

if [[ -z ${SITL_HOST} ]]; then
	echo "ERROR: 必须使用 --sitl-host 指定 PX4 SITL 所在机器的 IPv4 地址" >&2
	exit 2
fi

if [[ ! ${SITL_HOST} =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]]; then
	echo "ERROR: --sitl-host 必须是 IPv4 地址" >&2
	exit 2
fi

if [[ -z ${COUNT} ]]; then
	if [[ ! -f ${RUNTIME_DIR}/interfaces.tsv ]]; then
		echo "ERROR: 当前场景接口表不存在，请先启动场景，或使用 -n 指定架数" >&2
		exit 1
	fi
	COUNT=$(awk 'NR > 1 && NF { count++ } END { print count + 0 }' "${RUNTIME_DIR}/interfaces.tsv")
fi

if [[ ! ${COUNT} =~ ^[0-9]+$ ]] || ((COUNT < 1 || COUNT > 10)); then
	echo "ERROR: 架数必须是 1～10 的整数" >&2
	exit 2
fi

if [[ -z ${MISSION_BIN} ]]; then
	MISSION_BIN="${SCRIPT_DIR}/build/mavsdk_route_test"
	if [[ ! -x ${MISSION_BIN} || ${SCRIPT_DIR}/mavsdk_route_test.cpp -nt ${MISSION_BIN} ]]; then
		MAVSDK_ROUTE_TEST_BIN=${MISSION_BIN} bash "${SCRIPT_DIR}/build_mavsdk_route_test.sh" "${MAVSDK_PREFIX}"
	fi
	MAVSDK_LIB_DIR=${MAVSDK_LIB_DIR:-${MAVSDK_PREFIX}/lib}
fi

if [[ ! -x ${MISSION_BIN} ]]; then
	echo "ERROR: 航线测试程序不存在或不可执行：${MISSION_BIN}" >&2
	exit 1
fi

if [[ -n ${MAVSDK_LIB_DIR} && ! -f ${MAVSDK_LIB_DIR}/libmavsdk.so ]]; then
	echo "ERROR: ${MAVSDK_LIB_DIR}/libmavsdk.so 不存在" >&2
	exit 1
fi

for ((i = 0; i < COUNT; i++)); do
	aircraft_id=$((i + 1))
	url="udpout://${SITL_HOST}:$((14280 + i))"
	echo
	echo "飞机 ${aircraft_id}/${COUNT} 航线测试："
	printf '  command: '
	printf 'LD_LIBRARY_PATH=%q timeout 480 %q %q\n' \
		"${MAVSDK_LIB_DIR:-<程序内置RPATH>}" "${MISSION_BIN}" "${url}"
	if ! LD_LIBRARY_PATH="${MAVSDK_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
		timeout 480 "${MISSION_BIN}" "${url}"; then
		echo "FAIL: 飞机 ${aircraft_id} 的 MAVSDK 航线测试失败" >&2
		exit 1
	fi
done

echo
echo "PASS: ${COUNT} 架飞机均通过 MAVSDK UDP 完成航线、返航和落地验证"

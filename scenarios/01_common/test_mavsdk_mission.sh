#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PX4_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
MAVSDK_DIR=$(cd -- "${PX4_DIR}/.." && pwd)/MAVSDK
COUNT=4
SITL_HOST=127.0.0.1
MISSION_BIN=
MAVSDK_LIB_DIR=

usage() {
	cat <<'EOF'
Usage: test_mavsdk_mission.sh [-n COUNT] [--sitl-host HOST] [--mavsdk-dir PATH]
                               [--mission-bin PATH] [--mavsdk-lib-dir PATH]

Fly the native MAVSDK fly_mission example for each running PX4 instance.
MAVSDK must be desneo/MAVSDK branch v4.0.6-udp-dyt and already built.

Options:
  -n, --count COUNT   Number of vehicles, 1-10 (default: 4)
      --sitl-host     PX4 SITL IPv4 address (default: 127.0.0.1)
      --mavsdk-dir    MAVSDK source directory (default: ../MAVSDK)
      --mission-bin   Prebuilt mission-test application
      --mavsdk-lib-dir Directory containing the matching libmavsdk.so
  -h, --help          Show this help
EOF
}

while (($# > 0)); do
	case "$1" in
		-n|--count)
			(($# >= 2)) || { echo "ERROR: $1 requires a value" >&2; exit 2; }
			COUNT=$2
			shift 2
			;;
		--mavsdk-dir)
			(($# >= 2)) || { echo "ERROR: $1 requires a value" >&2; exit 2; }
			MAVSDK_DIR=$(realpath "$2")
			shift 2
			;;
		--sitl-host)
			(($# >= 2)) || { echo "ERROR: $1 requires a value" >&2; exit 2; }
			SITL_HOST=$2
			shift 2
			;;
		--mission-bin)
			(($# >= 2)) || { echo "ERROR: $1 requires a value" >&2; exit 2; }
			MISSION_BIN=$(realpath "$2")
			shift 2
			;;
		--mavsdk-lib-dir)
			(($# >= 2)) || { echo "ERROR: $1 requires a value" >&2; exit 2; }
			MAVSDK_LIB_DIR=$(realpath "$2")
			shift 2
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			echo "ERROR: unknown argument: $1" >&2
			usage >&2
			exit 2
			;;
	esac
done

if [[ ! ${COUNT} =~ ^[0-9]+$ ]] || ((COUNT < 1 || COUNT > 10)); then
	echo "ERROR: COUNT must be an integer from 1 to 10" >&2
	exit 2
fi

if [[ ! ${SITL_HOST} =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]]; then
	echo "ERROR: --sitl-host must be an IPv4 address" >&2
	exit 2
fi

if [[ -z ${MISSION_BIN} ]]; then
	if [[ ! -d ${MAVSDK_DIR}/.git ]]; then
		echo "ERROR: MAVSDK repository not found at ${MAVSDK_DIR}" >&2
		exit 1
	fi

	branch=$(git -C "${MAVSDK_DIR}" branch --show-current)
	if [[ ${branch} != v4.0.6-udp-dyt ]]; then
		echo "ERROR: expected MAVSDK branch v4.0.6-udp-dyt, found '${branch}'" >&2
		exit 1
	fi

	MISSION_BIN="${MAVSDK_DIR}/build/fly_mission/fly_mission"
	MAVSDK_LIB_DIR="${MAVSDK_DIR}/install/lib"
fi

if [[ ! -x ${MISSION_BIN} ]]; then
	echo "ERROR: mission test executable not found: ${MISSION_BIN}" >&2
	exit 1
fi

if [[ -n ${MAVSDK_LIB_DIR} && ! -f ${MAVSDK_LIB_DIR}/libmavsdk.so ]]; then
	echo "ERROR: libmavsdk.so not found in ${MAVSDK_LIB_DIR}" >&2
	exit 1
fi

for ((i = 0; i < COUNT; i++)); do
	url="udpout://${SITL_HOST}:$((14280 + i))"
	echo "MAVSDK[${i}] command: LD_LIBRARY_PATH=${MAVSDK_LIB_DIR:-<embedded-rpath>} ${MISSION_BIN} ${url}"
	if ! LD_LIBRARY_PATH="${MAVSDK_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
		timeout 300 "${MISSION_BIN}" "${url}"; then
		echo "FAIL: MAVSDK mission failed for PX4 instance ${i}" >&2
		exit 1
	fi
done

echo "PASS: ${COUNT} vehicle(s) completed missions through native MAVSDK over UDP"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
DEFAULT_PREFIX=/home/firefly/code/3rdparty/common/mavsdk/v3.17.4/arm64/ubuntu22.04
PREFIX=${1:-${MAVSDK_PREFIX:-${DEFAULT_PREFIX}}}
OUTPUT=${MAVSDK_MISSION_BIN:-${SCRIPT_DIR}/build/mavsdk_mission}

if [[ ! -f ${PREFIX}/include/mavsdk/mavsdk.h || ! -f ${PREFIX}/lib/libmavsdk.so ]]; then
	echo "ERROR: prebuilt MAVSDK package not found under ${PREFIX}" >&2
	exit 1
fi

mkdir -p "$(dirname -- "${OUTPUT}")"

echo "Compiling test application only; MAVSDK library remains unchanged."
echo "c++ -std=c++17 -O2 -I${PREFIX}/include -I${PREFIX}/include/mavsdk ${SCRIPT_DIR}/mavsdk_mission.cpp -L${PREFIX}/lib -Wl,-rpath,${PREFIX}/lib -lmavsdk -pthread -o ${OUTPUT}"
c++ -std=c++17 -O2 -I"${PREFIX}/include" -I"${PREFIX}/include/mavsdk" \
	"${SCRIPT_DIR}/mavsdk_mission.cpp" \
	-L"${PREFIX}/lib" -Wl,-rpath,"${PREFIX}/lib" -lmavsdk -pthread \
	-o "${OUTPUT}"

echo "Built ${OUTPUT}"

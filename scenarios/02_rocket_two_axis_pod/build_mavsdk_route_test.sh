#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PX4_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
DEFAULT_PREFIX=$(cd -- "${PX4_DIR}/.." && pwd)/MAVSDK/install
PREFIX=${1:-${MAVSDK_PREFIX:-${DEFAULT_PREFIX}}}
OUTPUT=${MAVSDK_ROUTE_TEST_BIN:-${SCRIPT_DIR}/build/mavsdk_route_test}

if [[ ! -f ${PREFIX}/include/mavsdk/mavsdk.h && ! -f ${PREFIX}/include/mavsdk/mavsdk.hpp ]]; then
	echo "ERROR: 找不到 MAVSDK 头文件：${PREFIX}/include/mavsdk" >&2
	exit 1
fi

if [[ ! -f ${PREFIX}/lib/libmavsdk.so ]]; then
	echo "ERROR: 找不到预编译 MAVSDK：${PREFIX}" >&2
	echo "请传入 MAVSDK 安装前缀；本脚本只编译测试程序，不编译 MAVSDK 库。" >&2
	exit 1
fi

mkdir -p "$(dirname -- "${OUTPUT}")"

echo "仅编译航线测试程序，不重新编译或修改 MAVSDK 库。"
printf 'c++ -std=c++17 -O2 -I%q -I%q %q -L%q -Wl,-rpath,%q -lmavsdk -pthread -o %q\n' \
	"${PREFIX}/include" "${PREFIX}/include/mavsdk" "${SCRIPT_DIR}/mavsdk_route_test.cpp" \
	"${PREFIX}/lib" "${PREFIX}/lib" "${OUTPUT}"
c++ -std=c++17 -O2 -I"${PREFIX}/include" -I"${PREFIX}/include/mavsdk" \
	"${SCRIPT_DIR}/mavsdk_route_test.cpp" \
	-L"${PREFIX}/lib" -Wl,-rpath,"${PREFIX}/lib" -lmavsdk -pthread \
	-o "${OUTPUT}"

echo "已生成：${OUTPUT}"

#!/usr/bin/env bash
set -euo pipefail

# ---------------------------
# Usage: ./compile_dts.sh [w5500|w6100]
# Default: w6100
# ---------------------------

usage() {
    echo "Usage: $0 [w5500|w6100]"
    exit 1
}

# 인자 처리 (없으면 기본값 w6100)
CHIP="${1:-w6100}"

case "$CHIP" in
    w5500|w6100)
        ;;  # 정상 값이면 통과
    *)
        usage
        ;;
esac

# 경로 설정
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DTS_SRC="${SCRIPT_DIR}/wizchip-driver.dts"
BUILD_DIR="${SCRIPT_DIR}/../output"
TMP_DTS="${BUILD_DIR}/wizchip-driver.${CHIP}.dts"
DTBO_OUT="${BUILD_DIR}/${CHIP}-driver.dtbo"

mkdir -p "${BUILD_DIR}"

# compatible 값 치환
# "compatible = "wiznet,xxxx";" 부분을 인자로 받은 CHIP으로 바꿔줌
sed -E 's/(compatible[[:space:]]*=[[:space:]]*"wiznet,)[^"]+(")/\1'"${CHIP}"'\2/' \
    "${DTS_SRC}" > "${TMP_DTS}"

# 컴파일 (디바이스 트리 오버레이용이라 -@ 옵션 추가)
dtc -@ -I dts -O dtb -o "${DTBO_OUT}" "${TMP_DTS}"

echo "✔ Built: ${DTBO_OUT}"
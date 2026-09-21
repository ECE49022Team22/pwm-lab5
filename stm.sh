#!/usr/bin/env bash
# Build, upload, and monitor the STM32 nucleo_f091rc firmware.
#
# Usage:
#   ./stm.sh              # build, upload, then open serial monitor
#   ./stm.sh build        # build only
#   ./stm.sh upload       # build + upload only
#   ./stm.sh monitor      # open serial monitor only
#
# Env overrides:
#   PIO_PORT=/dev/ttyUSB0 ./stm.sh monitor   # force a specific serial port

set -euo pipefail

ENV_NAME="nucleo_f091rc"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PIO_BIN="$HOME/.platformio/penv/bin/platformio"
if ! command -v "$PIO_BIN" >/dev/null 2>&1; then
    PIO_BIN="$(command -v platformio || command -v pio || true)"
fi
if [[ -z "$PIO_BIN" ]]; then
    echo "error: platformio not found (expected at ~/.platformio/penv/bin/platformio)" >&2
    exit 1
fi

ACTION="${1:-all}"

build() {
    "$PIO_BIN" run --environment "$ENV_NAME"
}

upload() {
    "$PIO_BIN" run --environment "$ENV_NAME" --target upload
}

monitor() {
    local port_args=()
    if [[ -n "${PIO_PORT:-}" ]]; then
        port_args=(--port "$PIO_PORT")
    fi
    "$PIO_BIN" device monitor "${port_args[@]}"
}

case "$ACTION" in
    build)   build ;;
    upload)  build; upload ;;
    monitor) monitor ;;
    all)     build; upload; monitor ;;
    *)
        echo "usage: $0 [build|upload|monitor|all]" >&2
        exit 1
        ;;
esac

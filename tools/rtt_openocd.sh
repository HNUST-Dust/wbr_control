#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
app_dir="$(cd "${script_dir}/.." && pwd)"
workspace_dir="$(cd "${app_dir}/.." && pwd)"
build_dir="${1:-${app_dir}/build}"

if [[ "${build_dir}" != /* ]]; then
    build_dir="${app_dir}/${build_dir}"
fi

elf_file="${build_dir}/zephyr/zephyr.elf"
openocd="${workspace_dir}/../openocd-hpm-rtt-v0.1.0/local-rtt/bin/openocd"
hpm_openocd="${workspace_dir}/../sdk_env/hpm_sdk/boards/openocd"
rtt_port="${RTT_PORT:-9090}"
rtt_halt_polling="${RTT_HALT_POLLING:-off}"
rtt_poll_interval_ms="${RTT_POLL_INTERVAL_MS:-100}"

if [[ "${rtt_halt_polling}" != "on" && "${rtt_halt_polling}" != "off" ]]; then
    echo "RTT_HALT_POLLING must be 'on' or 'off'" >&2
    exit 1
fi

if [[ ! "${rtt_poll_interval_ms}" =~ ^[1-9][0-9]*$ ]]; then
    echo "RTT_POLL_INTERVAL_MS must be a positive integer" >&2
    exit 1
fi

if [[ ! -x "${openocd}" ]]; then
    echo "Patched HPM OpenOCD not found: ${openocd}" >&2
    exit 1
fi

if [[ ! -f "${elf_file}" ]]; then
    echo "Zephyr ELF not found: ${elf_file}" >&2
    echo "Usage: $0 [build-directory]" >&2
    exit 1
fi

rtt_address="$(nm -n "${elf_file}" | awk '$3 == "_SEGGER_RTT" && !address { address = "0x" $1 } END { print address }')"
if [[ ! "${rtt_address}" =~ ^0x[0-9A-Fa-f]+$ ]]; then
    echo "Unable to find _SEGGER_RTT in ${elf_file}" >&2
    exit 1
fi

echo "RTT control block: ${rtt_address}"
echo "RTT channel 0 will listen on TCP port ${rtt_port}."
echo "RTT access mode: halt-polling=${rtt_halt_polling}, interval=${rtt_poll_interval_ms} ms"
if [[ "${WEST_RTT_AUTO_ATTACH:-0}" != "1" ]]; then
    echo "In another terminal run: nc 127.0.0.1 ${rtt_port}"
fi

exec "${openocd}" \
    -s "${hpm_openocd}" \
    -f "${hpm_openocd}/probes/cmsis_dap.cfg" \
    -f "${hpm_openocd}/soc/hpm6750-single-core.cfg" \
    -f "${hpm_openocd}/boards/hpm6750evk2.cfg" \
    -c init \
    -c "riscv set_mem_access sysbus" \
    -c "riscv virt2phys_mode off" \
    -c "hpm6750.cpu0 rtt halt_polling ${rtt_halt_polling}" \
    -c "hpm6750.cpu0 rtt setup ${rtt_address} 0x100 \"SEGGER RTT\"" \
    -c "hpm6750.cpu0 rtt polling_interval ${rtt_poll_interval_ms}" \
    -c "hpm6750.cpu0 rtt start" \
    -c "rtt server start ${rtt_port} 0" \
    -c "debug_level 1"

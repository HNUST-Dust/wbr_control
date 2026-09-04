#!/usr/bin/env python3
## @file usb_interrupt_perf.py
#  @brief 测量 CherryUSB 厂商中断端点的吞吐量与传输稳定性。
#  @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。

from __future__ import annotations

import argparse
import statistics
import struct
import sys
import time

try:
    import usb.core
    import usb.util
except ImportError:
    usb = None


VID = 0x34B7
PID = 0xFFFF
INTF = 0
EP_OUT = 0x01
EP_IN = 0x81
MAGIC = 0x49544E54
HEADER = struct.Struct("<IIIIII")
CMD_ECHO = 0
CMD_STREAM_IN_START = 1
CMD_STREAM_IN_STOP = 2
CMD_STREAM_OUT_START = 3
CMD_STREAM_OUT_STOP = 4
CMD_STREAM_OUT_DATA = 5
CMD_RESET = 6


def checksum32(data: bytes) -> int:
    value = 0
    for byte in data:
        value = ((value << 5) | (value >> 27)) & 0xFFFFFFFF
        value = (value + byte) & 0xFFFFFFFF
    return value


def open_device() -> usb.core.Device:
    if usb is None:
        print("pyusb is required: python3 -m pip install pyusb", file=sys.stderr)
        raise SystemExit(2)

    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        raise RuntimeError(f"device {VID:04x}:{PID:04x} not found")

    dev.set_configuration()
    try:
        if dev.is_kernel_driver_active(INTF):
            dev.detach_kernel_driver(INTF)
    except (NotImplementedError, usb.core.USBError):
        pass

    usb.util.claim_interface(dev, INTF)
    return dev


def endpoint_mps(dev: usb.core.Device, ep_addr: int) -> int:
    cfg = dev.get_active_configuration()
    intf = cfg[(INTF, 0)]
    ep = usb.util.find_descriptor(
        intf,
        custom_match=lambda endpoint: endpoint.bEndpointAddress == ep_addr,
    )
    if ep is None:
        raise RuntimeError(f"endpoint 0x{ep_addr:02x} not found")
    return int(ep.wMaxPacketSize)


def make_frame(seq: int, size: int, cmd: int = CMD_ECHO) -> bytes:
    if size < HEADER.size:
        raise ValueError(f"packet size must be at least {HEADER.size}")

    payload_len = size - HEADER.size
    if payload_len:
        payload = bytes([cmd]) + bytes(((seq + i) & 0xFF for i in range(1, payload_len)))
    else:
        payload = b""
    prefix = HEADER.pack(MAGIC, seq, time.perf_counter_ns() // 1000 & 0xFFFFFFFF,
                         cmd, payload_len, 0) + payload
    crc = checksum32(prefix[:20])
    return prefix[:20] + struct.pack("<I", crc) + payload


def hex_preview(data: bytes, limit: int = 32) -> str:
    return " ".join(f"{byte:02x}" for byte in data[:limit])


def parse_frame(data: bytes, expected_cmd: int | None = None) -> tuple[int, int, int, int]:
    if len(data) < HEADER.size:
        raise RuntimeError(f"short response: {len(data)} bytes")

    magic, seq, host_us, device_ms, payload_len, crc = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise RuntimeError(f"bad magic: 0x{magic:08x}")
    if crc != checksum32(data[:20]):
        raise RuntimeError(f"bad crc: got=0x{crc:08x}")
    if payload_len > len(data) - HEADER.size:
        raise RuntimeError(f"bad payload_len: {payload_len} for {len(data)} bytes")
    payload_cmd = data[HEADER.size] if payload_len > 0 and len(data) > HEADER.size else host_us
    cmd = payload_cmd if payload_cmd == host_us or expected_cmd == payload_cmd else host_us
    if expected_cmd is not None and cmd != expected_cmd:
        raise RuntimeError(
            f"cmd mismatch: expected={expected_cmd} header_cmd={host_us} "
            f"payload_cmd={payload_cmd} seq={seq} preview={hex_preview(data)}"
        )
    return seq, cmd, device_ms, payload_len


def print_ms_stats(name: str, samples: list[float]) -> None:
    ordered = sorted(samples)
    p90 = ordered[int(len(ordered) * 0.90)]
    p99 = ordered[int(len(ordered) * 0.99)]
    print(f"{name}: min={min(samples):.3f} ms avg={statistics.mean(samples):.3f} ms "
          f"p50={statistics.median(samples):.3f} ms p90={p90:.3f} ms "
          f"p99={p99:.3f} ms max={max(samples):.3f} ms")


def drain_in(dev: usb.core.Device, read_size: int, quiet_reads: int = 3, max_reads: int = 32) -> int:
    drained = 0
    quiet = 0
    reads = 0
    while quiet < quiet_reads and reads < max_reads:
        try:
            data = dev.read(EP_IN, read_size, timeout=20)
        except usb.core.USBTimeoutError:
            quiet += 1
            continue
        drained += len(data)
        reads += 1
        quiet = 0
    return drained


def reset_device_mode(dev: usb.core.Device, size: int, read_size: int) -> None:
    drain_in(dev, read_size, quiet_reads=1)
    dev.write(EP_OUT, make_frame(0, size, CMD_RESET), timeout=1000)
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        try:
            data = bytes(dev.read(EP_IN, read_size, timeout=50))
        except usb.core.USBTimeoutError:
            break
        try:
            parse_frame(data, expected_cmd=CMD_RESET)
            break
        except RuntimeError:
            continue
    drain_in(dev, read_size, quiet_reads=1)


def run_ping(dev: usb.core.Device, size: int) -> None:
    read_size = max(size, endpoint_mps(dev, EP_IN))
    frame = make_frame(0, size)
    start = time.perf_counter_ns()
    written = dev.write(EP_OUT, frame, timeout=1000)
    response = bytes(dev.read(EP_IN, read_size, timeout=1000))
    end = time.perf_counter_ns()
    seq, _, device_ms, _ = parse_frame(response, expected_cmd=CMD_ECHO)
    print(f"written={written} read={len(response)} seq={seq} "
          f"device_ms={device_ms} rtt_ms={(end - start) / 1_000_000:.3f}")


def run_latency(dev: usb.core.Device, count: int, size: int) -> None:
    read_size = max(size, endpoint_mps(dev, EP_IN))
    samples = []

    for seq in range(count):
        frame = make_frame(seq, size)
        start = time.perf_counter_ns()
        dev.write(EP_OUT, frame, timeout=1000)
        response = bytes(dev.read(EP_IN, read_size, timeout=1000))
        end = time.perf_counter_ns()
        got_seq, _, _, _ = parse_frame(response, expected_cmd=CMD_ECHO)
        if got_seq != seq:
            raise RuntimeError(f"seq mismatch: expected={seq} got={got_seq}")
        samples.append((end - start) / 1_000_000)

    print(f"interrupt_latency count={count} size={size}")
    print_ms_stats("rtt", samples)


def run_exchange(dev: usb.core.Device, count: int, size: int) -> None:
    read_size = max(size, endpoint_mps(dev, EP_IN))
    start = time.perf_counter()
    bytes_out = 0
    bytes_in = 0

    for seq in range(count):
        frame = make_frame(seq, size)
        bytes_out += dev.write(EP_OUT, frame, timeout=1000)
        response = bytes(dev.read(EP_IN, read_size, timeout=1000))
        bytes_in += len(response)
        got_seq, _, _, _ = parse_frame(response, expected_cmd=CMD_ECHO)
        if got_seq != seq:
            raise RuntimeError(f"seq mismatch: expected={seq} got={got_seq}")

    elapsed = time.perf_counter() - start
    print(f"interrupt_exchange count={count} size={size} elapsed={elapsed:.3f}s "
          f"out={bytes_out / elapsed / 1024:.1f} KiB/s "
          f"in={bytes_in / elapsed / 1024:.1f} KiB/s")


def run_stream_in(dev: usb.core.Device, count: int, size: int) -> None:
    read_size = max(size, endpoint_mps(dev, EP_IN))
    reset_device_mode(dev, size, read_size)
    dev.write(EP_OUT, make_frame(0, size, CMD_STREAM_IN_START), timeout=1000)

    seq_errors = 0
    intervals_ms = []
    bytes_in = 0
    last_ns = None
    start = time.perf_counter_ns()

    try:
        for expected in range(count):
            data = bytes(dev.read(EP_IN, read_size, timeout=1000))
            now = time.perf_counter_ns()
            bytes_in += len(data)
            seq, cmd, _, _ = parse_frame(data, expected_cmd=CMD_STREAM_IN_START)
            if seq != expected or cmd != CMD_STREAM_IN_START:
                seq_errors += 1
            if last_ns is not None:
                intervals_ms.append((now - last_ns) / 1_000_000)
            last_ns = now
    finally:
        try:
            dev.write(EP_OUT, make_frame(0, size, CMD_STREAM_IN_STOP), timeout=1000)
        finally:
            reset_device_mode(dev, size, read_size)

    elapsed = (time.perf_counter_ns() - start) / 1_000_000_000
    print(f"interrupt_stream_in count={count} size={size} read_size={read_size} elapsed={elapsed:.3f}s "
          f"rate={count / elapsed:.1f} fps in={bytes_in / elapsed / 1024:.1f} KiB/s "
          f"seq_errors={seq_errors}")
    if intervals_ms:
        print_ms_stats("interval", intervals_ms)


def run_stream_out(dev: usb.core.Device, count: int, size: int) -> None:
    read_size = max(size, endpoint_mps(dev, EP_IN))
    reset_device_mode(dev, size, read_size)
    dev.write(EP_OUT, make_frame(0, size, CMD_STREAM_OUT_START), timeout=1000)

    start = time.perf_counter()
    bytes_out = 0
    try:
        for seq in range(count):
            bytes_out += dev.write(EP_OUT, make_frame(seq, size, CMD_STREAM_OUT_DATA), timeout=1000)
        elapsed = time.perf_counter() - start

        dev.write(EP_OUT, make_frame(count, size, CMD_STREAM_OUT_STOP), timeout=1000)
        deadline = time.monotonic() + 1.0
        status = "no_stop_response=timeout"
        while time.monotonic() < deadline:
            try:
                response = bytes(dev.read(EP_IN, read_size, timeout=100))
                device_count, cmd, _, _ = parse_frame(response, expected_cmd=CMD_STREAM_OUT_STOP)
                status = f"device_count={device_count} cmd={cmd}"
                break
            except usb.core.USBTimeoutError:
                break
            except RuntimeError as exc:
                status = f"ignored_stale={exc}"
                continue
    finally:
        reset_device_mode(dev, size, read_size)

    print(f"interrupt_stream_out count={count} size={size} read_size={read_size} elapsed={elapsed:.3f}s "
          f"rate={count / elapsed:.1f} fps out={bytes_out / elapsed / 1024:.1f} KiB/s "
          f"{status}")


def main() -> None:
    parser = argparse.ArgumentParser(description="CherryUSB vendor interrupt endpoint test")
    parser.add_argument("--size", type=int, default=64,
                        help="interrupt packet size, use 64 for FS-style or up to 1024 on HS")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping")
    latency = sub.add_parser("latency")
    latency.add_argument("--count", type=int, default=200)
    exchange = sub.add_parser("exchange")
    exchange.add_argument("--count", type=int, default=1000)
    stream_in = sub.add_parser("stream-in")
    stream_in.add_argument("--count", type=int, default=2000)
    stream_out = sub.add_parser("stream-out")
    stream_out.add_argument("--count", type=int, default=2000)

    args = parser.parse_args()
    dev = open_device()
    try:
        if args.cmd == "ping":
            run_ping(dev, args.size)
        elif args.cmd == "latency":
            run_latency(dev, args.count, args.size)
        elif args.cmd == "exchange":
            run_exchange(dev, args.count, args.size)
        elif args.cmd == "stream-in":
            run_stream_in(dev, args.count, args.size)
        elif args.cmd == "stream-out":
            run_stream_out(dev, args.count, args.size)
    finally:
        usb.util.release_interface(dev, INTF)


if __name__ == "__main__":
    main()

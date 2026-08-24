#!/usr/bin/env python3
## @file usb_cdc_perf.py
#  @brief 测量 CherryUSB CDC ACM 链路的吞吐量与传输稳定性。
#  @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。

import argparse
import statistics
import sys
import time

try:
    import serial
except ImportError:
    serial = None


DEFAULT_BAUD = 115200
ECHO_SETTLE_S = 0.2
SYNC_TIMEOUT_S = 4.0
SYNC_RETRY_S = 1.0
READY_TIMEOUT_S = 4.0
STATUS_PREFIXES = ("ECHO_READY", "RX_READY", "RX_DONE", "TX_READY", "PONG", "ERR")
_READ_BACKLOG = bytearray()


def open_serial(path: str, baud: int) -> "serial.Serial":
    if serial is None:
        print("pyserial is required: python3 -m pip install pyserial", file=sys.stderr)
        raise SystemExit(2)

    ser = serial.Serial(
        path,
        baudrate=baud,
        timeout=0.01,
        write_timeout=1.0,
        exclusive=True,
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def write_all(ser: "serial.Serial", data: bytes) -> None:
    view = memoryview(data)
    while view:
        written = ser.write(view)
        if written is None:
            written = len(view)
        if written == 0:
            raise TimeoutError("timed out waiting for serial write readiness")
        view = view[written:]
    ser.flush()


def sync_to_echo(ser: "serial.Serial") -> None:
    drain(ser)
    deadline = time.monotonic() + SYNC_TIMEOUT_S
    next_probe = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_probe:
            write_all(ser, b"$ECHO\n")
            next_probe = now + SYNC_RETRY_S

        try:
            line = read_line(ser, min(0.05, max(0.0, deadline - time.monotonic())))
        except TimeoutError:
            continue

        if line.startswith("ECHO_READY"):
            break
        if any(line.startswith(prefix) for prefix in STATUS_PREFIXES):
            continue
    else:
        raise TimeoutError("timed out waiting for ECHO_READY")

    time.sleep(ECHO_SETTLE_S)
    drain(ser)


def drain_status_lines(ser: "serial.Serial", quiet_s: float = 0.1, max_s: float = 1.0) -> None:
    quiet_deadline = time.monotonic() + quiet_s
    hard_deadline = time.monotonic() + max_s

    while time.monotonic() < hard_deadline:
        try:
            line = read_line(ser, min(0.02, max(0.0, hard_deadline - time.monotonic())))
        except TimeoutError:
            if time.monotonic() >= quiet_deadline:
                return
            continue

        if any(line.startswith(prefix) for prefix in STATUS_PREFIXES):
            quiet_deadline = time.monotonic() + quiet_s
            continue

        _READ_BACKLOG[:0] = line.encode("ascii", errors="replace")
        return


def read_some(ser: "serial.Serial", timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting:
            return ser.read(min(waiting, 65536))

        data = ser.read(1)
        if data:
            waiting = ser.in_waiting
            if waiting:
                data += ser.read(min(waiting, 65535))
            return data
    return b""


def read_exact(ser: "serial.Serial", size: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    chunks = []
    remaining = size
    while remaining > 0 and time.monotonic() < deadline:
        if _READ_BACKLOG:
            data = bytes(_READ_BACKLOG)
            _READ_BACKLOG.clear()
        else:
            data = read_some(ser, max(0.0, deadline - time.monotonic()))
        if not data:
            break
        chunk = data[:remaining]
        chunks.append(chunk)
        remaining -= len(chunk)
        if len(data) > len(chunk):
            _READ_BACKLOG.extend(data[len(chunk):])
    result = b"".join(chunks)
    if len(result) != size:
        raise TimeoutError(f"expected {size} bytes, got {len(result)} bytes")
    return result


def read_line(ser: "serial.Serial", timeout_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    line = bytearray()
    while time.monotonic() < deadline:
        newline = _READ_BACKLOG.find(b"\n")
        if newline >= 0:
            line += _READ_BACKLOG[:newline + 1]
            del _READ_BACKLOG[:newline + 1]
            before_newline = line.split(b"\n", 1)[0]
            return before_newline.decode("ascii", errors="replace").strip()

        if _READ_BACKLOG:
            line += _READ_BACKLOG
            _READ_BACKLOG.clear()
            continue

        data = read_some(ser, max(0.0, deadline - time.monotonic()))
        if not data:
            break
        _READ_BACKLOG.extend(data)
    raise TimeoutError("timed out waiting for line")


def read_status_line(ser: "serial.Serial", expected: str, timeout_s: float,
                     ignored: tuple[str, ...] = ()) -> str:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(ser, max(0.0, deadline - time.monotonic()))
        if line.startswith(expected):
            return line
        if any(line.startswith(prefix) for prefix in ignored):
            continue
        raise RuntimeError(f"unexpected response: {line}")
    raise TimeoutError(f"timed out waiting for {expected}")


def write_command_until_ready(ser: "serial.Serial", command: bytes, expected: str,
                              ignored: tuple[str, ...] = ()) -> str:
    deadline = time.monotonic() + READY_TIMEOUT_S
    write_all(ser, command)

    while time.monotonic() < deadline:
        try:
            line = read_line(ser, min(0.05, max(0.0, deadline - time.monotonic())))
        except TimeoutError:
            continue

        if line.startswith(expected):
            return line
        if any(line.startswith(prefix) for prefix in ignored):
            continue
        raise RuntimeError(f"unexpected response: {line}")

    raise TimeoutError(f"timed out waiting for {expected}")


def drain(ser: "serial.Serial") -> None:
    _READ_BACKLOG.clear()
    while read_some(ser, 0.05):
        pass


def mbps(byte_count: int, elapsed_s: float) -> float:
    return byte_count / elapsed_s / (1024 * 1024)


def run_latency(ser: "serial.Serial", count: int) -> None:
    samples_ms = []

    print("selecting echo mode...", flush=True)
    sync_to_echo(ser)
    drain_status_lines(ser)
    print("device: ECHO_READY", flush=True)

    print(f"running latency test: count={count} command=$PING", flush=True)
    for seq in range(count):
        command = f"$PING {seq}\n".encode("ascii")
        start = time.perf_counter_ns()
        write_all(ser, command)
        response = read_status_line(ser, "PONG", 2.0, ignored=("ECHO_READY",))
        end = time.perf_counter_ns()
        samples_ms.append((end - start) / 1_000_000)

    print(f"latency command_ping count={count}")
    print(f"min={min(samples_ms):.3f} ms avg={statistics.mean(samples_ms):.3f} ms "
          f"p50={statistics.median(samples_ms):.3f} ms "
          f"max={max(samples_ms):.3f} ms")


def run_rx(ser: "serial.Serial", total_bytes: int, chunk_size: int) -> None:
    pattern = bytes((i & 0xFF for i in range(chunk_size)))

    sync_to_echo(ser)
    print(f"arming RX sink: bytes={total_bytes}", flush=True)
    ready = write_command_until_ready(
        ser,
        f"$RX {total_bytes}\n".encode("ascii"),
        "RX_READY",
        ignored=("ECHO_READY", "TX_READY", "RX_DONE", "PONG"),
    )
    print(ready, flush=True)

    start = time.perf_counter()
    remaining = total_bytes
    while remaining > 0:
        chunk = pattern[:min(chunk_size, remaining)]
        write_all(ser, chunk)
        remaining -= len(chunk)
    done = read_status_line(ser, "RX_DONE", 5.0, ignored=("RX_READY", "ECHO_READY"))
    elapsed = time.perf_counter() - start

    print(done)
    print(f"host_to_device={mbps(total_bytes, elapsed):.2f} MiB/s "
          f"bytes={total_bytes} host_elapsed={elapsed:.3f} s")


def run_tx(ser: "serial.Serial", total_bytes: int) -> None:
    sync_to_echo(ser)
    print(f"starting TX stream: bytes={total_bytes}", flush=True)
    write_command_until_ready(
        ser,
        f"$TX {total_bytes}\n".encode("ascii"),
        "TX_READY",
        ignored=("ECHO_READY", "RX_READY", "RX_DONE", "PONG"),
    )
    start = time.perf_counter()
    read_exact(ser, total_bytes, 10.0)
    elapsed = time.perf_counter() - start
    print(f"device_to_host={mbps(total_bytes, elapsed):.2f} MiB/s "
          f"bytes={total_bytes} host_elapsed={elapsed:.3f} s")


def run_ping(ser: "serial.Serial") -> None:
    sync_to_echo(ser)
    write_all(ser, b"$PING\n")
    response = read_status_line(ser, "PONG", 2.0, ignored=("ECHO_READY", "TX_READY", "RX_READY", "RX_DONE"))
    print(response)


def main() -> None:
    parser = argparse.ArgumentParser(description="CherryUSB CDC ACM speed and latency tool")
    parser.add_argument("port", help="serial device, for example /dev/tty.usbmodemXXXX or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)

    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ping")

    latency = sub.add_parser("latency")
    latency.add_argument("--count", type=int, default=200)

    rx = sub.add_parser("rx")
    rx.add_argument("--bytes", type=int, default=4 * 1024 * 1024)
    rx.add_argument("--chunk", type=int, default=256 * 1024)

    tx = sub.add_parser("tx")
    tx.add_argument("--bytes", type=int, default=4 * 1024 * 1024)

    args = parser.parse_args()

    ser = open_serial(args.port, args.baud)
    try:
        print(f"opened {args.port}", flush=True)
        drain(ser)
        if args.cmd == "ping":
            run_ping(ser)
        elif args.cmd == "latency":
            run_latency(ser, args.count)
        elif args.cmd == "rx":
            run_rx(ser, args.bytes, args.chunk)
        elif args.cmd == "tx":
            run_tx(ser, args.bytes)
    finally:
        ser.close()


if __name__ == "__main__":
    main()

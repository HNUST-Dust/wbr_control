# SPDX-License-Identifier: Apache-2.0

"""West extension for the wbr_control HPM OpenOCD RTT console."""

import argparse
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import threading
from textwrap import dedent

from west import log
from west.commands import WestCommand


class Rtt(WestCommand):
    def __init__(self):
        super().__init__(
            "rtt",
            "open the wbr_control RTT console",
            dedent(
                """
                Start the patched HPM OpenOCD RTT server and attach an
                interactive terminal to Zephyr RTT channel 0.

                The default build directory is wbr_control/build. Press
                Ctrl-C to close the terminal and stop OpenOCD.
                """
            ),
        )

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            description=self.description,
            formatter_class=argparse.RawDescriptionHelpFormatter,
        )
        parser.add_argument(
            "-d",
            "--build-dir",
            type=Path,
            help="Zephyr build directory (default: wbr_control/build)",
        )
        parser.add_argument(
            "-p",
            "--port",
            type=self._port,
            default=9090,
            help="local RTT TCP port (default: 9090)",
        )
        parser.add_argument(
            "--timeout",
            type=float,
            default=10.0,
            help="seconds to wait for the RTT server (default: 10)",
        )
        parser.add_argument(
            "--server-only",
            action="store_true",
            help="start the RTT server without attaching a terminal",
        )
        parser.add_argument(
            "--halt-polling",
            action="store_true",
            help=(
                "briefly halt the CPU for each RTT poll instead of using "
                "the default non-intrusive SBA access"
            ),
        )
        parser.add_argument(
            "--poll-interval",
            type=int,
            default=100,
            metavar="MS",
            help="RTT polling interval in milliseconds (default: 100)",
        )
        return parser

    @staticmethod
    def _port(value):
        port = int(value)
        if not 1 <= port <= 65535:
            raise argparse.ArgumentTypeError("port must be in range 1..65535")
        return port

    @staticmethod
    def _send_signal(process, sig, process_group=False):
        """Signal a child, optionally including its isolated process group."""
        try:
            if process_group:
                os.killpg(process.pid, sig)
            else:
                process.send_signal(sig)
        except (OSError, ProcessLookupError):
            # The child may have exited between poll() and signal delivery.
            pass

    @classmethod
    def _stop_process(cls, process, process_group=False):
        if process is None or process.poll() is not None:
            return

        cls._send_signal(process, signal.SIGINT, process_group)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            cls._send_signal(process, signal.SIGTERM, process_group)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                cls._send_signal(process, signal.SIGKILL, process_group)
                process.wait()

    @classmethod
    def _stop_openocd(cls, process):
        if process is None or process.poll() is not None:
            return

        # Stop the periodic RTT callback before shutting OpenOCD down. In the
        # HPM6750 fallback mode each completed poll resumes the target.
        try:
            with socket.create_connection(("127.0.0.1", 4444), timeout=1) as control:
                control.sendall(
                    b"hpm6750.cpu0 rtt stop\n"
                    b"shutdown\n"
                )
            process.wait(timeout=3)
            return
        except (OSError, subprocess.TimeoutExpired):
            # The launcher is created with start_new_session=True. Kill its
            # entire process group so a future launcher change cannot leave a
            # USB-owning OpenOCD descendant behind.
            cls._stop_process(process, process_group=True)

    @staticmethod
    def _ignore_cleanup_signals():
        """Prevent a second Ctrl-C/SIGHUP from interrupting USB cleanup."""
        previous = {}
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[sig] = signal.signal(sig, signal.SIG_IGN)
        return previous

    @staticmethod
    def _restore_signals(previous):
        for sig, handler in previous.items():
            signal.signal(sig, handler)

    def do_run(self, args, _unknown_args):
        app_dir = Path(__file__).resolve().parents[2]
        launcher = app_dir / "tools" / "rtt_openocd.sh"
        build_dir = (
            args.build_dir.expanduser().resolve()
            if args.build_dir
            else app_dir / "build"
        )

        if not launcher.is_file():
            log.die(f"RTT launcher not found: {launcher}")
        if args.timeout <= 0:
            log.die("--timeout must be greater than zero")
        if args.poll_interval <= 0:
            log.die("--poll-interval must be greater than zero")

        netcat = shutil.which("nc")
        if not args.server_only and netcat is None:
            log.die("nc was not found; install netcat or use --server-only")

        env = os.environ.copy()
        env["RTT_PORT"] = str(args.port)
        env["WEST_RTT_AUTO_ATTACH"] = "1"
        env["RTT_HALT_POLLING"] = "on" if args.halt_polling else "off"
        env["RTT_POLL_INTERVAL_MS"] = str(args.poll_interval)

        server = None
        terminal = None
        ready = threading.Event()
        startup_failed = threading.Event()
        stopping = threading.Event()

        def drain_openocd():
            assert server is not None
            assert server.stdout is not None

            marker = f"Listening on port {args.port} for rtt connections"
            for raw_line in server.stdout:
                line = raw_line.rstrip()
                if marker in line:
                    ready.set()

                if (
                    not stopping.is_set()
                    and (not ready.is_set() or "Error:" in line or "Warn :" in line)
                ):
                    log.inf(line)

            if not ready.is_set():
                startup_failed.set()

        try:
            server = subprocess.Popen(
                [os.fspath(launcher), os.fspath(build_dir)],
                cwd=app_dir,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            reader = threading.Thread(target=drain_openocd, daemon=True)
            reader.start()

            if not ready.wait(args.timeout):
                if startup_failed.is_set() or server.poll() is not None:
                    log.die("OpenOCD exited before the RTT server was ready")
                log.die(
                    f"timed out after {args.timeout:g}s waiting for RTT "
                    f"port {args.port}"
                )

            if args.server_only:
                log.inf(f"RTT server ready: nc 127.0.0.1 {args.port}")
                server.wait()
                return

            log.inf(f"RTT connected on 127.0.0.1:{args.port}; press Ctrl-C to exit")
            terminal = subprocess.Popen(
                [netcat, "127.0.0.1", str(args.port)],
            )
            return_code = terminal.wait()
            if return_code != 0:
                log.wrn(f"RTT terminal exited with status {return_code}")
        except KeyboardInterrupt:
            pass
        finally:
            stopping.set()
            # Ctrl-C is often pressed more than once when a terminal appears
            # slow to exit. Do not let the second signal abort cleanup between
            # closing netcat and releasing the CMSIS-DAP libusb interface.
            previous_signals = self._ignore_cleanup_signals()
            try:
                self._stop_process(terminal)
                self._stop_openocd(server)
                if server is not None and server.poll() is None:
                    log.err("OpenOCD did not exit; the debugger USB interface may still be busy")
                else:
                    log.inf("RTT stopped; OpenOCD released the debugger interface")
            finally:
                self._restore_signals(previous_signals)

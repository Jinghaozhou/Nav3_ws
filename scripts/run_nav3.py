#!/usr/bin/env python3
"""Run Nav3 in its own process group and clean up children on exit/signals."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def group_exists(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def signal_group(pgid, sig):
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        pass


def main():
    if any(arg in ('-h', '--help') for arg in sys.argv[1:]):
        print(__doc__)
        print('Usage: python3 scripts/run_nav3.py [nav3.launch.py arguments]')
        print('Example: python3 scripts/run_nav3.py driver:=false chassis:=true')
        return 0
    root = Path(__file__).resolve().parents[1]
    requested_signal = None

    def request_stop(sig, _frame):
        nonlocal requested_signal
        requested_signal = sig

    # Install handlers before starting the child so shutdown during startup is handled.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)
    process = subprocess.Popen(
        ['bash', '-c', 'source "$1" || exit $?; shift; exec ros2 launch nav3_bringup nav3.launch.py "$@"',
         'nav3', str(root / 'scripts/source_env.bash'), *sys.argv[1:]],
        cwd=root, start_new_session=True)
    print(f'[Nav3 runner] launch PID/PGID={process.pid}', flush=True)
    try:
        while process.poll() is None and requested_signal is None:
            time.sleep(.1)
    finally:
        # Let launch forward the first SIGINT itself. Sending it to both launch and
        # its children would deliver duplicate interrupts during Python cleanup.
        for sig, grace in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
            if not group_exists(process.pid):
                break
            if sig == signal.SIGINT and process.poll() is None:
                try:
                    process.send_signal(sig)
                except ProcessLookupError:
                    pass
            else:
                signal_group(process.pid, sig)
            deadline = time.monotonic() + grace
            while group_exists(process.pid) and time.monotonic() < deadline:
                process.poll()  # Reap the launch parent even while waiting for descendants.
                time.sleep(.1)
        process.wait()
    if requested_signal is not None:
        return 128 + requested_signal
    return process.returncode if process.returncode >= 0 else 128 - process.returncode


if __name__ == '__main__':
    raise SystemExit(main())

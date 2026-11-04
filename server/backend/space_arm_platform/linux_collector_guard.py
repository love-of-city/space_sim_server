"""Private Linux collector supervisor, launched by collection_processes.

Use a separate interpreter, not preexec_fn in the threaded API. Linux subreaper
adoption retains ownership even when a worker exits or a descendant calls
setsid(). Only descendants of this helper are signalled; no global PID/group
search or privileged service is needed.
"""

from __future__ import annotations

import ctypes
import os
import select
import signal
import subprocess
import sys
import time

import psutil


def become_subreaper():
    libc = ctypes.CDLL(None, use_errno=True)
    prctl = libc.prctl
    prctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong,
                      ctypes.c_ulong, ctypes.c_ulong]
    prctl.restype = ctypes.c_int
    if prctl(36, 1, 0, 0, 0) != 0:  # PR_SET_CHILD_SUBREAPER
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))


def reap_exited_children():
    while True:
        try:
            pid, _ = os.waitpid(-1, os.WNOHANG)
        except ChildProcessError:
            return
        if pid == 0:
            return


def reap_orphans(worker_pid):
    # A long batch may leave short-lived adopted children (e.g. crash reporters).
    # Reap them while it runs, without stealing the main worker's exit status
    # from Popen.poll(). Only direct children are waitable here.
    for child in psutil.Process().children():
        if child.pid != worker_pid:
            try:
                os.waitpid(child.pid, os.WNOHANG)
            except ChildProcessError:
                pass


def close_descendants():
    owner = psutil.Process()
    deadline = time.monotonic() + 10
    while True:
        children = owner.children(recursive=True)
        # Freeze before killing. Any fork racing with this snapshot is adopted
        # here when its parent dies, and is included in the next sweep.
        for child in children:
            try:
                child.suspend()
            except psutil.NoSuchProcess:
                pass
        for child in reversed(children):
            try:
                child.kill()
            except psutil.NoSuchProcess:
                pass
        reap_exited_children()
        if not owner.children(recursive=True):
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("Collector descendants did not exit during cleanup")
        time.sleep(.02)


def run(owner_fd, command):
    become_subreaper()  # Fail before launching a worker if the OS rejects this.
    os.set_inheritable(owner_fd, False)
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        # Admission also protects an immediately exiting command from the
        # coordinator's PID/create-time lookup racing with helper exit.
        while not stopping:
            ready, _, _ = select.select([owner_fd], [], [], .1)
            if ready:
                if os.read(owner_fd, 1) != b"G":
                    return 125
                break
        if stopping:
            return 125
        process = subprocess.Popen(command)
        while not stopping:
            returncode = process.poll()
            if returncode is not None:
                return returncode
            reap_orphans(process.pid)
            ready, _, _ = select.select([owner_fd], [], [], .1)
            if ready and not os.read(owner_fd, 1):
                stopping = True
        return 125
    finally:
        # Stay alive as the subreaper until every owned process has been reaped.
        close_descendants()
        os.close(owner_fd)


if __name__ == "__main__":
    try:
        code = run(int(sys.argv[1]), sys.argv[2:])
    except Exception as error:
        print(f"Collector supervisor failed: {type(error).__name__}: {error}",
              file=sys.stderr, flush=True)
        code = 127
    if code < 0:
        # Preserve Popen's negative signal exit status for the real worker.
        if -code not in {signal.SIGKILL, signal.SIGSTOP}:
            signal.signal(-code, signal.SIG_DFL)
        os.kill(os.getpid(), -code)
    raise SystemExit(code)

# nebulaos_memory - klippy-side memory policy, host-side Klipper extra
#
# NebulaOS production-kernel plan, MEM-T09. klippy runs at oom_score_adj -900
# (set at birth by S55klipper), and a forked child inherits that value. The
# heavy forked work - input-shaper calibration and other multiprocessing
# helpers that Klipper starts with the fork start method - must not inherit
# klippy's protection: an OOM kill of such a helper fails one calibration,
# while losing klippy aborts a print. An at-fork hook resets every forked
# child to child_oom_score_adj (default 300, the optional-work tier).
#
# subprocess.Popen children do not run at-fork hooks; nebulaos-memguard's
# periodic scan resets any of those that keep a negative value.
#
# MEM-T10: optional mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT) of klippy,
# so its pages are never swapped or reclaimed while a print runs (a major
# fault in the reactor delays every MCU message behind it). ONFAULT locks
# pages as they are touched instead of populating every mapping up front.
# "mlock: off" (the module default) makes no call; platform.cfg enables it,
# and hardware measurement (HWDP-MEM-5) decides whether it stays. Locks are
# not inherited by forked children. The call is idempotent across RESTART.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os

_FORK_HOOK_INSTALLED = False

# <sys/mman.h>; identical on MIPS (arch/mips/include/uapi/asm/mman.h)
MCL_CURRENT, MCL_FUTURE, MCL_ONFAULT = 1, 2, 4
MLOCK_ONFAULT = MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT


def _mlockall(flags):
    """Returns (rc, errno). Never raises: a failure is reported, not fatal."""
    try:
        import cffi
        ffi = cffi.FFI()
        ffi.cdef("int mlockall(int flags);")
        rc = ffi.dlopen(None).mlockall(flags)
        return rc, (ffi.errno if rc else 0)
    except Exception:
        logging.exception("nebulaos_memory: mlockall unavailable")
        return -1, -1


def _child_reset(adj, path="/proc/self/oom_score_adj"):
    try:
        with open(path, "w") as f:
            f.write(str(adj))
    except OSError:
        pass


class NebulaOSMemory:
    def __init__(self, config):
        global _FORK_HOOK_INSTALLED
        self.child_adj = config.getint(
            "child_oom_score_adj", 300, minval=0, maxval=1000)
        # klippy RESTART re-instantiates the object in the same process, and
        # at-fork hooks are process-global: install exactly once.
        if not _FORK_HOOK_INSTALLED:
            adj = self.child_adj
            os.register_at_fork(after_in_child=lambda: _child_reset(adj))
            _FORK_HOOK_INSTALLED = True
        self.status = {"child_oom_score_adj": self.child_adj}
        mode = config.getchoice("mlock", {"off": 0, "onfault": MLOCK_ONFAULT}, "off")
        if mode:
            rc, err = _mlockall(mode)
            logging.info("nebulaos_memory: mlockall(0x%x) rc=%d errno=%d", mode, rc, err)
            self.status.update(mlock_flags=mode, mlock_rc=rc, mlock_errno=err)

    def get_status(self, eventtime):
        return dict(self.status)


def load_config(config):
    return NebulaOSMemory(config)

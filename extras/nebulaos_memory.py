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
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os

_FORK_HOOK_INSTALLED = False


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

    def get_status(self, eventtime):
        return dict(self.status)


def load_config(config):
    return NebulaOSMemory(config)

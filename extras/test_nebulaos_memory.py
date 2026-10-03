# nebulaos_memory tests: the at-fork child adj reset (MEM-T09).
#
# Run from this directory: python3 -m unittest test_nebulaos_memory -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import tempfile
import unittest
from unittest import mock

try:
    from . import nebulaos_memory
except ImportError:
    import nebulaos_memory


class FakeConfig(object):
    def __init__(self, values=None):
        self._values = values or {}

    def getint(self, key, default=None, minval=None, maxval=None):
        v = int(self._values.get(key, default))
        if minval is not None and v < minval:
            raise ValueError(key)
        if maxval is not None and v > maxval:
            raise ValueError(key)
        return v

    def getchoice(self, key, choices, default=None):
        v = self._values.get(key, default)
        return choices[v]


class ChildResetTest(unittest.TestCase):
    def setUp(self):
        nebulaos_memory._FORK_HOOK_INSTALLED = False

    def test_child_reset_writes_value(self):
        with tempfile.NamedTemporaryFile("w+", delete=False) as f:
            path = f.name
        try:
            nebulaos_memory._child_reset(300, path)
            with open(path) as f:
                self.assertEqual(f.read(), "300")
        finally:
            os.unlink(path)

    def test_child_reset_ignores_oserror(self):
        nebulaos_memory._child_reset(300, "/nonexistent/dir/oom_score_adj")

    def test_hook_installed_once_across_restarts(self):
        with mock.patch.object(os, "register_at_fork") as reg:
            nebulaos_memory.NebulaOSMemory(FakeConfig())
            nebulaos_memory.NebulaOSMemory(FakeConfig())
            self.assertEqual(reg.call_count, 1)

    def test_status(self):
        with mock.patch.object(os, "register_at_fork"):
            m = nebulaos_memory.NebulaOSMemory(
                FakeConfig({"child_oom_score_adj": 250}))
        self.assertEqual(m.get_status(0)["child_oom_score_adj"], 250)

    def test_forked_child_gets_the_value(self):
        # Real fork with the real hook; the child reports through a pipe what
        # it wrote (raising oom_score_adj needs no privilege).
        try:
            with open("/proc/self/oom_score_adj") as f:
                mine = int(f.read())
        except OSError:
            self.skipTest("no /proc/self/oom_score_adj")
        want = min(1000, max(mine, 0) + 300)
        nebulaos_memory.NebulaOSMemory(FakeConfig({"child_oom_score_adj": want}))
        r, w = os.pipe()
        pid = os.fork()
        if pid == 0:
            os.close(r)
            with open("/proc/self/oom_score_adj") as f:
                os.write(w, f.read().strip().encode())
            os._exit(0)
        os.close(w)
        got = os.read(r, 32).decode()
        os.close(r)
        os.waitpid(pid, 0)
        self.assertEqual(int(got), want)
        with open("/proc/self/oom_score_adj") as f:
            self.assertEqual(int(f.read()), mine)


if __name__ == "__main__":
    unittest.main()

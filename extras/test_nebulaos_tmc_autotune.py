# Tests for nebulaos_tmc_autotune (decision D8, 2026-10-10) and the vendored
# motor database. Run from klippy/: python3 -m unittest extras.test_nebulaos_tmc_autotune
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import unittest

from . import nebulaos_tmc_autotune as nta

KE_TMC = {"tmc2208 stepper_x", "tmc2208 stepper_y", "tmc2208 stepper_z",
          "stepper_x", "stepper_y", "stepper_z", "extruder"}


class FakeGCmd:
    def __init__(self, params):
        self.p = params
        self.info = []

    def get(self, name, default=None):
        return self.p.get(name, default)

    def get_int(self, name, default=None, minval=None, maxval=None):
        return int(self.p.get(name, default))

    def respond_info(self, msg):
        self.info.append(msg)

    def error(self, msg):
        return Exception(msg)


class FakeConfigfile:
    def __init__(self):
        self.sets = []
        self.removed = []

    def get_status(self, eventtime):
        return {"settings": {k: {} for k in KE_TMC}, "config": {}}

    def set(self, section, option, value):
        self.sets.append((section, option, value))

    def remove_section(self, section):
        self.removed.append(section)


class FakePrinter:
    def __init__(self):
        self.commands = {}
        self.objects = {"configfile": FakeConfigfile(), "gcode": self}

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def register_command(self, name, cb, desc=None):
        self.commands[name] = cb


class FakeConfig:
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer


class MotorDatabaseTests(unittest.TestCase):
    def test_ke_stock_motors_present(self):
        names = nta.motor_names()
        self.assertIn("creality-42-34", names)
        self.assertIn("creality-42-40", names)

    def test_database_sits_next_to_the_module(self):
        self.assertTrue(os.path.isfile(nta.MOTOR_DATABASE))


class ValidationTests(unittest.TestCase):
    motors = {"creality-42-34", "creality-42-40"}

    def test_valid(self):
        self.assertEqual(nta.validate_selection("stepper_x", "creality-42-34",
                                                "performance", KE_TMC, self.motors), "")

    def test_extruder_has_no_tmc(self):
        self.assertIn("no TMC", nta.validate_selection("extruder", "creality-42-34",
                                                       "auto", KE_TMC, self.motors))

    def test_unknown_motor(self):
        self.assertIn("unknown motor", nta.validate_selection(
            "stepper_x", "nope", "auto", KE_TMC, self.motors))

    def test_bad_goal(self):
        self.assertIn("GOAL", nta.validate_selection(
            "stepper_x", "creality-42-34", "fast", KE_TMC, self.motors))

    def test_injection_refused(self):
        self.assertIn("STEPPER", nta.validate_selection(
            "stepper_x]\n[x", "creality-42-34", "auto", KE_TMC, self.motors))


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.printer = FakePrinter()
        self.mod = nta.load_config(FakeConfig(self.printer))
        self.cfg = self.printer.objects["configfile"]

    def test_registered(self):
        self.assertIn("NEBULAOS_TMC_AUTOTUNE", self.printer.commands)

    def test_stages_section(self):
        g = FakeGCmd({"STEPPER": "stepper_z", "MOTOR": "creality-42-40", "GOAL": "silent"})
        self.mod.cmd_NEBULAOS_TMC_AUTOTUNE(g)
        self.assertEqual(self.cfg.sets, [("autotune_tmc stepper_z", "motor", "creality-42-40"),
                                         ("autotune_tmc stepper_z", "tuning_goal", "silent")])
        self.assertIn("SAVE_CONFIG", g.info[0])

    def test_invalid_stages_nothing(self):
        with self.assertRaises(Exception):
            self.mod.cmd_NEBULAOS_TMC_AUTOTUNE(FakeGCmd({"STEPPER": "stepper_x", "MOTOR": "nope"}))
        self.assertEqual(self.cfg.sets, [])

    def test_remove(self):
        self.mod.cmd_NEBULAOS_TMC_AUTOTUNE(FakeGCmd({"STEPPER": "stepper_y", "REMOVE": 1}))
        self.assertEqual(self.cfg.removed, ["autotune_tmc stepper_y"])


if __name__ == "__main__":
    unittest.main()

# tmcstatus reactor-safety tests
#
# Regression test for the crash where get_status() performed synchronous UART
# reads (via reactor.pause()) inside a reactor.assert_no_pause() context,
# producing "Internal error - reactor pause disabled" for each TMC driver
# followed by a cascading 'NoneType' has no attribute 'timer_is_running'.
#
# The fix: get_status() returns cached data only; a reactor timer refreshes
# the cache in a context where reactor.pause() is allowed.
#
# Run from klippy/: python3 -m unittest extras.test_tmcstatus -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import unittest


class FakeFieldHelper:
    """Minimal TMC field helper with field/register lookup."""
    def __init__(self, fields=None, registers=None, drv_status_fields=None):
        self._fields = fields or {}
        self._registers = registers or {}
        self._drv_status_fields = drv_status_fields or {}

    def lookup_register(self, field, default='_missing_'):
        if field in self._registers:
            return self._registers[field]
        if default != '_missing_':
            return default
        raise KeyError(field)

    def get_field(self, field):
        return self._fields.get(field, 0)

    def get_reg_fields(self, reg, val):
        return dict(self._drv_status_fields)


class FakeMCUTMC:
    """Minimal TMC UART/SPI mock that tracks get_register calls."""
    def __init__(self, fail=False):
        self.read_count = 0
        self._fail = fail

    def get_register(self, reg):
        self.read_count += 1
        if self._fail:
            raise Exception("Internal error - reactor pause disabled")
        return 0x00000000


class FakeTMCDriver:
    """Minimal TMC driver object."""
    def __init__(self, mcu_tmc=None, fields=None):
        self.mcu_tmc = mcu_tmc or FakeMCUTMC()
        self.fields = fields or FakeFieldHelper(
            registers={'sg_result': 'SG_RESULT'},
            drv_status_fields={'ola': 0, 'olb': 0})


class FakeTimer:
    def __init__(self):
        self.callbacks = []
        self.cancelled = []
        self._time = 0.0

    def register_timer(self, cb, waketime):
        handle = len(self.callbacks)
        self.callbacks.append(cb)
        return handle

    def unregister_timer(self, handle):
        self.cancelled.append(handle)

    def monotonic(self):
        return self._time

    NEVER = -1.0


class FakePrinter:
    def __init__(self):
        self._objects = {}
        self._handlers = {}
        self._reactor = FakeTimer()

    def register_event_handler(self, name, cb):
        self._handlers.setdefault(name, []).append(cb)

    def lookup_object(self, name):
        return self._objects[name]

    def get_reactor(self):
        return self._reactor

    def fire(self, event):
        for cb in self._handlers.get(event, []):
            cb()


class FakeConfig:
    def __init__(self, printer, prefix_sections=None):
        self._printer = printer
        self._prefix_sections = prefix_sections or {}

    def get_printer(self):
        return self._printer

    def get_prefix_sections(self, prefix):
        return self._prefix_sections.get(prefix, [])


class FakeSection:
    def __init__(self, name, sense_resistor=0.110):
        self._name = name
        self._sense_resistor = sense_resistor

    def get_name(self):
        return self._name

    def getfloat(self, key, default=None):
        if key == 'sense_resistor':
            return self._sense_resistor
        return default


class TestTMCStatusReactorSafety(unittest.TestCase):
    """The fix must ensure get_status() never calls mcu_tmc.get_register()."""

    def _make_status(self, drivers=None, fail_reads=False):
        from . import tmcstatus

        printer = FakePrinter()
        sections = []
        if drivers is None:
            drivers = ['tmc2208 stepper_x', 'tmc2208 stepper_y',
                       'tmc2208 stepper_z']
        for d in drivers:
            prefix = d.split()[0]
            sec = FakeSection(d)
            sections.append((prefix, sec))
            mcu_tmc = FakeMCUTMC(fail=fail_reads)
            driver_obj = FakeTMCDriver(mcu_tmc=mcu_tmc)
            printer._objects[d] = driver_obj

        prefix_map = {}
        for prefix, sec in sections:
            prefix_map.setdefault(prefix, []).append(sec)

        config = FakeConfig(printer, prefix_map)
        status = tmcstatus.TMCStatus(config)
        return status, printer

    def test_get_status_returns_empty_before_connect(self):
        status, printer = self._make_status()
        result = status.get_status(0.0)
        self.assertEqual(result, {})

    def test_get_status_returns_cached_after_refresh(self):
        status, printer = self._make_status()
        printer.fire('klippy:connect')
        status._refresh_cache()
        result = status.get_status(0.0)
        self.assertIn('tmc2208 stepper_x', result)
        self.assertIn('tmc2208 stepper_y', result)
        self.assertIn('tmc2208 stepper_z', result)

    def test_get_status_does_not_read_registers(self):
        """The exact regression: get_status must NOT call get_register."""
        status, printer = self._make_status()
        printer.fire('klippy:connect')
        status._refresh_cache()

        for d in ['tmc2208 stepper_x', 'tmc2208 stepper_y',
                   'tmc2208 stepper_z']:
            printer._objects[d].mcu_tmc.read_count = 0

        status.get_status(0.0)

        for d in ['tmc2208 stepper_x', 'tmc2208 stepper_y',
                   'tmc2208 stepper_z']:
            self.assertEqual(printer._objects[d].mcu_tmc.read_count, 0,
                             "%s: get_status must not read registers" % d)

    def test_failed_refresh_preserves_last_good_cache(self):
        status, printer = self._make_status()
        printer.fire('klippy:connect')
        status._refresh_cache()
        self.assertIn('tmc2208 stepper_x', status.get_status(0.0))

        for d in printer._objects:
            printer._objects[d].mcu_tmc._fail = True

        status._refresh_cache()
        result = status.get_status(0.0)
        self.assertIn('tmc2208 stepper_x', result)

    def test_get_status_safe_during_simulated_pause_disabled(self):
        """Simulates the exact crash scenario: reads fail with 'reactor pause
        disabled', but get_status still returns the last cached data."""
        status, printer = self._make_status(fail_reads=True)
        printer.fire('klippy:connect')
        status._refresh_cache()
        result = status.get_status(0.0)
        self.assertEqual(result, {})

    def test_shutdown_stops_timer(self):
        status, printer = self._make_status()
        printer.fire('klippy:connect')
        self.assertIsNotNone(status._refresh_timer)
        printer.fire('klippy:shutdown')
        self.assertTrue(status._shutdown)
        self.assertIsNone(status._refresh_timer)

    def test_timer_returns_never_after_shutdown(self):
        status, printer = self._make_status()
        printer.fire('klippy:connect')
        printer.fire('klippy:shutdown')
        result = status._timer_refresh(0.0)
        self.assertEqual(result, FakeTimer.NEVER)

    def test_multiple_rapid_status_queries(self):
        """Multiple rapid get_status calls must not trigger hardware reads."""
        status, printer = self._make_status()
        printer.fire('klippy:connect')
        status._refresh_cache()

        for d in printer._objects:
            printer._objects[d].mcu_tmc.read_count = 0

        for _ in range(100):
            status.get_status(0.0)

        for d in printer._objects:
            self.assertEqual(printer._objects[d].mcu_tmc.read_count, 0,
                             "rapid queries must not cause hardware reads")

    def test_startup_no_drivers(self):
        status, printer = self._make_status(drivers=[])
        printer.fire('klippy:connect')
        status._refresh_cache()
        self.assertEqual(status.get_status(0.0), {})

    def test_irms_calculation(self):
        from . import tmcstatus
        printer = FakePrinter()
        mcu_tmc = FakeMCUTMC()
        fields = FakeFieldHelper(
            fields={'vsense': 0, 'hstrt': 3, 'hend': 2},
            registers={'sg_result': 'SG_RESULT'},
            drv_status_fields={'cs_actual': 16, 'ola': 0})
        driver = FakeTMCDriver(mcu_tmc=mcu_tmc, fields=fields)
        printer._objects['tmc2208 stepper_x'] = driver
        sec = FakeSection('tmc2208 stepper_x', sense_resistor=0.110)
        config = FakeConfig(printer, {'tmc2208': [sec]})
        status = tmcstatus.TMCStatus(config)
        printer.fire('klippy:connect')
        status._refresh_cache()
        result = status.get_status(0.0)
        self.assertIn('tmc2208 stepper_x', result)
        data = result['tmc2208 stepper_x']
        self.assertIn('i_rms', data)
        self.assertGreater(data['i_rms'], 0)


if __name__ == '__main__':
    unittest.main()

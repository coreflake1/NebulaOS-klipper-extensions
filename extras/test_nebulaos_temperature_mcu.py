# nebulaos_temperature_mcu tests - GD32 die-temperature support against REAL mainline Klipper.
#
# These tests deliberately import and drive official Klipper's own extras.heaters and
# extras.temperature_mcu rather than fakes of them. That is the whole point: the module under
# test is a subclass whose extension point is upstream's config_unknown() fall-through, and
# whose registration goes through upstream's add_sensor_factory()/setup_sensor(). Faking
# either side would test the fake. Only the hardware-facing edges (ADC pin, MCU constants,
# printer object graph) are stubbed, and each stub models exactly one real interface.
#
# Because they run against the composed tree's real Klipper, these tests also fail loudly if a
# future Klipper pin removes config_unknown() as the no-match fall-through, or changes
# add_sensor_factory()/setup_sensor() - which is precisely the drift nebulaos_compat.py exists
# to catch at runtime.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_temperature_mcu -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import itertools
import unittest

from . import heaters
from . import nebulaos_temperature_mcu


_SENTINEL = object()


class _FakeGCode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, handler, desc=None, when_not_ready=False):
        self.commands[name] = handler


class _FakeMCUAdc:
    """Models the object ppins.setup_pin('adc', ...) returns, to the extent
    PrinterTemperatureMCU uses it."""

    def __init__(self, mcu):
        self._mcu = mcu
        self.callback = None
        self.sample_calls = []

    def get_mcu(self):
        return self._mcu

    def setup_adc_callback(self, callback):
        self.callback = callback

    def setup_adc_sample(self, report_time, sample_time, sample_count,
                         minval=None, maxval=None, range_check_count=0):
        self.sample_calls.append((minval, maxval))


class _FakeDebugReadCmd:
    """mainline handle_mcu_identify() looks this command up unconditionally, before any chip
    dispatch, and the STM32/SAMD families then use it to read on-die factory calibration.

    The lookup must therefore succeed - and on this printer it genuinely does: the Klipper
    dictionaries embedded in this project's GD32F303xe firmware blobs declare exactly
    'debug_read order=%c addr=%u' / 'debug_result val=%u', the same strings mainline asks for.

    Sending it is a different matter. GD32 curves are datasheet constants, not on-die values,
    so any actual send would mean this test had silently wandered onto an STM32 code path.
    """

    def send(self, args):
        raise AssertionError(
            "a GD32 curve must be a pure datasheet constant - an on-die calibration read "
            "means this test exercised an STM32/SAMD path by mistake")


class _FakeMCU:
    """Models the MCU surface temperature_mcu.py touches: a chip-type constant and the
    debug_read query command it looks up before dispatching on chip type."""

    def __init__(self, mcu_type):
        self._mcu_type = mcu_type
        self.query_lookups = []

    def get_name(self):
        return 'mcu'

    def get_constants(self):
        return {'MCU': self._mcu_type}

    def lookup_query_command(self, msgformat, respformat):
        self.query_lookups.append((msgformat, respformat))
        return _FakeDebugReadCmd()


class _FakePins:
    def __init__(self, mcu):
        self._mcu = mcu
        self.setup_calls = []

    def setup_pin(self, pin_type, pin_name):
        self.setup_calls.append((pin_type, pin_name))
        return _FakeMCUAdc(self._mcu)


class _FakeQueryAdc:
    def register_adc(self, name, mcu_adc):
        pass


class _FakeErrorMCU:
    def add_clarify(self, msg, cb):
        pass


class _ConfigError(Exception):
    pass


class _FakeConfig:
    error = _ConfigError

    def __init__(self, values, name, printer):
        self._values = dict(values)
        self._name = name
        self._printer = printer

    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._name

    def get(self, option, default=_SENTINEL):
        if option in self._values:
            return self._values[option]
        if default is not _SENTINEL:
            return default
        raise _ConfigError("missing option %s" % option)

    def getfloat(self, option, default=_SENTINEL, minval=None, maxval=None):
        if option in self._values:
            return float(self._values[option])
        if default is not _SENTINEL:
            return default
        raise _ConfigError("missing option %s" % option)


class _FakePrinter:
    def __init__(self, mcu_type='gd32f303xe'):
        self.objects = {}
        self.event_handlers = {}
        self.mcu = _FakeMCU(mcu_type)
        self.objects['pins'] = _FakePins(self.mcu)
        self.objects['gcode'] = _FakeGCode()
        self.objects['query_adc'] = _FakeQueryAdc()
        self.objects['error_mcu'] = _FakeErrorMCU()

    # -- Printer API used by the code under test -------------------------------------

    def lookup_object(self, name, default=_SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is not _SENTINEL:
            return default
        raise _ConfigError("Unknown config object '%s'" % name)

    def load_object(self, config, name):
        """Real Printer.load_object() semantics for the two things that matter here: it is
        idempotent, and it constructs the object on first request."""
        if name not in self.objects:
            if name == 'heaters':
                self.objects[name] = heaters.PrinterHeaters(
                    _FakeConfig({}, 'heaters', self))
            else:
                raise _ConfigError("Unable to load module '%s'" % name)
        return self.objects[name]

    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)

    def send_event(self, event, *args):
        for cb in self.event_handlers.get(event, []):
            cb(*args)

    def get_start_args(self):
        return {}

    def config_error(self, msg):
        return _ConfigError(msg)


def _make_sensor(printer, section='temperature_sensor mcu_temp', values=None):
    """Build the sensor exactly as Klipper would: through the REAL PrinterHeaters.setup_sensor()
    dispatch, not by calling the class directly."""
    vals = {'sensor_type': nebulaos_temperature_mcu.SENSOR_TYPE}
    vals.update(values or {})
    config = _FakeConfig(vals, section, printer)
    pheaters = printer.load_object(config, 'heaters')
    # The default-sensor bootstrap reads upstream's temperature_sensors.cfg off disk through a
    # real configfile object; short-circuit it so this exercises factory dispatch alone.
    pheaters.have_load_sensors = True
    return pheaters.setup_sensor(config)


class FactoryRegistrationTest(unittest.TestCase):
    def test_registers_under_the_nebulaos_specific_sensor_type(self):
        printer = _FakePrinter()
        config = _FakeConfig({}, 'nebulaos_temperature_mcu', printer)
        pheaters = nebulaos_temperature_mcu.register_sensor_factory(printer, config)
        self.assertIs(pheaters.sensor_factories['nebulaos_temperature_mcu'],
                      nebulaos_temperature_mcu.NebulaOSTemperatureMCU)

    def test_does_not_shadow_upstreams_own_temperature_mcu_factory(self):
        # Registering under 'temperature_mcu' would make behaviour depend on which module
        # registered last - exactly the kind of ordering accident this module set must not
        # introduce.
        printer = _FakePrinter()
        config = _FakeConfig({}, 'nebulaos_temperature_mcu', printer)
        pheaters = nebulaos_temperature_mcu.register_sensor_factory(printer, config)
        self.assertNotIn('temperature_mcu', pheaters.sensor_factories)

    def test_registration_is_idempotent(self):
        printer = _FakePrinter()
        config = _FakeConfig({}, 'nebulaos_temperature_mcu', printer)
        for _ in range(3):
            pheaters = nebulaos_temperature_mcu.register_sensor_factory(printer, config)
        self.assertEqual(
            sum(1 for k in pheaters.sensor_factories if k.startswith('nebulaos')), 1)

    def test_load_config_registers_the_same_factory(self):
        printer = _FakePrinter()
        config = _FakeConfig({}, 'nebulaos_temperature_mcu', printer)
        nebulaos_temperature_mcu.load_config(config)
        pheaters = printer.lookup_object('heaters')
        self.assertIn('nebulaos_temperature_mcu', pheaters.sensor_factories)


class SectionOrderingTest(unittest.TestCase):
    """The determinism requirement, tested by construction rather than by hoping.

    heaters.setup_sensor() resolves sensor_type against a plain dict, so the factory must be
    registered before the [temperature_sensor] section is loaded. Klipper's only truly
    order-free bootstrap for that is klippy/extras/temperature_sensors.cfg, which NebulaOS does
    not patch. These tests therefore pin down both halves of the real contract: registration
    itself is order-independent (it works from any position, and repeatedly), and a genuinely
    too-late registration fails loudly rather than producing a mis-calibrated sensor.
    """

    def test_every_permutation_of_registration_before_use_succeeds(self):
        # Three independent registration triggers - the bare section's load_config(), the
        # explicit helper nebulaos_compat.py calls, and a redundant repeat - applied in every
        # possible order. All 6 permutations must leave a working factory. This is what makes
        # the result independent of section order rather than lucky in one arrangement.
        def via_load_config(printer):
            nebulaos_temperature_mcu.load_config(
                _FakeConfig({}, 'nebulaos_temperature_mcu', printer))

        def via_helper(printer):
            nebulaos_temperature_mcu.register_sensor_factory(
                printer, _FakeConfig({}, 'nebulaos_compat', printer))

        def via_repeat(printer):
            nebulaos_temperature_mcu.register_sensor_factory(
                printer, _FakeConfig({}, 'nebulaos_temperature_mcu', printer))

        triggers = (via_load_config, via_helper, via_repeat)
        permutations = list(itertools.permutations(triggers))
        self.assertEqual(len(permutations), 6)
        for order in permutations:
            printer = _FakePrinter()
            for trigger in order:
                trigger(printer)
            sensor = _make_sensor(printer)
            self.assertIsInstance(
                sensor, nebulaos_temperature_mcu.NebulaOSTemperatureMCU)

    def test_registration_before_the_sensor_section_resolves(self):
        printer = _FakePrinter()
        nebulaos_temperature_mcu.load_config(
            _FakeConfig({}, 'nebulaos_temperature_mcu', printer))
        sensor = _make_sensor(printer)
        self.assertIsInstance(sensor, nebulaos_temperature_mcu.NebulaOSTemperatureMCU)

    def test_without_registration_upstream_refuses_rather_than_guessing(self):
        # Klipper's own error, unmodified. It is loud and it stops startup - it does not fall
        # back to some other sensor. nebulaos_compat.py's preflight exists to turn this into a
        # message that names the ordering mistake; what matters here is that the failure mode
        # is a refusal.
        printer = _FakePrinter()
        with self.assertRaises(Exception) as ctx:
            _make_sensor(printer)
        self.assertIn('Unknown temperature sensor', str(ctx.exception))


class GD32CurveDispatchTest(unittest.TestCase):
    """Drives REAL mainline handle_mcu_identify(), so the whole upstream cfg_funcs loop runs
    and genuinely falls through to our config_unknown() override."""

    def _identify(self, mcu_type, values=None):
        printer = _FakePrinter(mcu_type=mcu_type)
        nebulaos_temperature_mcu.load_config(
            _FakeConfig({}, 'nebulaos_temperature_mcu', printer))
        sensor = _make_sensor(printer, values=values)
        sensor.setup_minmax(0., 100.)
        sensor.handle_mcu_identify()
        return sensor

    def test_all_three_gd32_chips_get_a_curve(self):
        for mcu_type in sorted(nebulaos_temperature_mcu.GD32_CURVES):
            sensor = self._identify(mcu_type)
            self.assertIsNotNone(sensor.slope, mcu_type)
            self.assertIsNotNone(sensor.base_temperature, mcu_type)

    def test_curve_values_match_the_shipped_fork(self):
        # The exact constants NebulaOS-klipper carried at KLIPPER_PIN 9ccb2e5d, recomputed
        # here from first principles rather than copied from the table under test.
        expected = {
            'gd32e230x8': 3.3 / -.004300,
            'gd32f303xe': 3.3 / -.004100,
            'gd32f303xb': 3.3 / -.004100,
        }
        for mcu_type, slope in expected.items():
            sensor = self._identify(mcu_type)
            self.assertAlmostEqual(sensor.slope, slope, places=9, msg=mcu_type)
            # base = calc_base(25 C, 1.45V/3.3V) = 25 - adc*slope
            self.assertAlmostEqual(
                sensor.base_temperature, 25. - (1.45 / 3.3) * slope,
                places=9, msg=mcu_type)

    def test_negative_slope_means_temperature_rises_as_adc_falls(self):
        # A sign error here would report the die cooling down as it heats up. Cheap to assert,
        # and the one property of these curves a reader can sanity-check without a datasheet.
        sensor = self._identify('gd32f303xe')
        self.assertLess(sensor.slope, 0.)
        self.assertGreater(sensor.calc_temp(0.30), sensor.calc_temp(0.50))

    def test_room_temperature_calibration_point_round_trips(self):
        # By construction the curve must report 25 C at 1.45 V on a 3.3 V reference.
        for mcu_type in sorted(nebulaos_temperature_mcu.GD32_CURVES):
            sensor = self._identify(mcu_type)
            self.assertAlmostEqual(sensor.calc_temp(1.45 / 3.3), 25., places=6,
                                   msg=mcu_type)

    def test_manual_override_still_wins_over_the_gd32_curve(self):
        # Inherited upstream behaviour: sensor_temperature1/sensor_adc1 are applied after the
        # cfg_funcs dispatch. Subclassing must not have broken that.
        sensor = self._identify('gd32f303xe',
                                values={'sensor_temperature1': '30.',
                                        'sensor_adc1': '0.5',
                                        'sensor_temperature2': '80.',
                                        'sensor_adc2': '0.25'})
        self.assertAlmostEqual(sensor.slope, (80. - 30.) / (0.25 - 0.5), places=9)
        self.assertAlmostEqual(sensor.calc_temp(0.5), 30., places=6)

    def test_unknown_non_gd32_chip_is_still_refused(self):
        printer = _FakePrinter(mcu_type='definitely_not_a_real_chip')
        nebulaos_temperature_mcu.load_config(
            _FakeConfig({}, 'nebulaos_temperature_mcu', printer))
        sensor = _make_sensor(printer)
        sensor.setup_minmax(0., 100.)
        with self.assertRaises(Exception) as ctx:
            sensor.handle_mcu_identify()
        msg = str(ctx.exception)
        self.assertIn('definitely_not_a_real_chip', msg)
        self.assertIn('gd32f303xe', msg)

    def test_prefix_matching_does_not_absorb_unrelated_gd32_variants(self):
        # Keys are matched with startswith(). A hypothetical future GD32 part that is not one
        # of the three qualified prefixes must NOT silently inherit a curve.
        printer = _FakePrinter(mcu_type='gd32l23x')
        nebulaos_temperature_mcu.load_config(
            _FakeConfig({}, 'nebulaos_temperature_mcu', printer))
        sensor = _make_sensor(printer)
        sensor.setup_minmax(0., 100.)
        with self.assertRaises(Exception):
            sensor.handle_mcu_identify()


class UpstreamContractTest(unittest.TestCase):
    """The assumptions this subclass makes about mainline, asserted rather than trusted."""

    def test_subclasses_upstreams_own_sensor(self):
        self.assertTrue(issubclass(nebulaos_temperature_mcu.NebulaOSTemperatureMCU,
                                   nebulaos_temperature_mcu.temperature_mcu
                                   .PrinterTemperatureMCU))

    def test_upstream_still_has_config_unknown_as_the_extension_point(self):
        self.assertTrue(hasattr(
            nebulaos_temperature_mcu.temperature_mcu.PrinterTemperatureMCU,
            'config_unknown'))

    def test_no_mainline_chip_prefix_can_shadow_a_gd32_chip_type(self):
        # If upstream ever added a prefix that a gd32* chip type starts with, dispatch would
        # never reach config_unknown() and these curves would go silently unused. Verified by
        # running upstream's REAL dispatch loop and recording whether it arrived here.
        for mcu_type in sorted(nebulaos_temperature_mcu.GD32_CURVES):
            printer = _FakePrinter(mcu_type=mcu_type)
            nebulaos_temperature_mcu.load_config(
                _FakeConfig({}, 'nebulaos_temperature_mcu', printer))
            sensor = _make_sensor(printer)
            sensor.setup_minmax(0., 100.)
            matched = {}
            original = sensor.config_unknown

            def record(_original=original, _matched=matched):
                _matched['hit'] = True
                _original()

            sensor.config_unknown = record
            sensor.handle_mcu_identify()
            self.assertTrue(
                matched.get('hit'),
                "a mainline chip prefix now shadows chip type %r - the GD32 curves would "
                "never be applied" % (mcu_type,))

    def test_upstream_still_looks_up_debug_read_before_dispatch(self):
        # Not a NebulaOS requirement, but a real prerequisite for this printer: if this
        # lookup ever became conditional or changed format, the composed tree's behaviour on
        # a GD32 would change. The KE's own dictionary declares this exact format.
        printer = _FakePrinter()
        nebulaos_temperature_mcu.load_config(
            _FakeConfig({}, 'nebulaos_temperature_mcu', printer))
        sensor = _make_sensor(printer)
        sensor.setup_minmax(0., 100.)
        sensor.handle_mcu_identify()
        self.assertIn(('debug_read order=%c addr=%u', 'debug_result val=%u'),
                      printer.mcu.query_lookups)

    def test_upstream_add_sensor_factory_is_still_a_plain_registration(self):
        printer = _FakePrinter()
        pheaters = printer.load_object(_FakeConfig({}, 'heaters', printer), 'heaters')
        marker = object()
        pheaters.add_sensor_factory('nebulaos_contract_probe', marker)
        self.assertIs(pheaters.sensor_factories['nebulaos_contract_probe'], marker)


if __name__ == '__main__':
    unittest.main()

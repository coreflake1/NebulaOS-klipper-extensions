# nebulaos_power_loss_recovery tests.
#
# Two layers, matching the module's own split:
#   - Pure-logic tests (PureLogicTests* below): drive meaningful_progress(),
#     build_sidecar_state(), validate_recovery(), build_resume_gcode_lines(),
#     atomic_write_json()/read_sidecar() directly with plain dicts and a
#     real tmp directory - zero Klipper printer objects involved at all.
#   - Glue tests (ExtensionStateMachineTests): drive the actual
#     NebulaOSPowerLossRecovery class against fakes of the printer object
#     graph (gcode, reactor, print_stats, virtual_sdcard, ...) and a
#     real in-memory fake EEPROM (io.BytesIO) plus a real tmp sidecar
#     directory - the two genuine hardware/filesystem boundaries are
#     faked, nothing about this module's own logic is.
#
# Run from klippy/: python3 -m unittest extras.test_nebulaos_power_loss_recovery -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import io
import json
import os
import shutil
import tempfile
import unittest

from . import nebulaos_plr_journal as journal
from . import nebulaos_power_loss_recovery as plr

Coord = collections.namedtuple('Coord', ['x', 'y', 'z', 'e'])


def _gcode_move_status(z=10.0, e=5.0, abs_coord=True, abs_extrude=True):
    return {
        'gcode_position': Coord(1.0, 2.0, z, e),
        'absolute_coordinates': abs_coord,
        'absolute_extrude': abs_extrude,
        'speed_factor': 1.0,
        'extrude_factor': 1.0,
        'homing_origin': Coord(0.0, 0.0, 0.5, 0.0),
    }


def _toolhead_status(z=10.0):
    return {
        'position': Coord(1.0, 2.0, z, 0.0),
        'max_velocity': 500.0,
        'max_accel': 8000.0,
        'minimum_cruise_ratio': 0.6875,
        'square_corner_velocity': 5.0,
    }


def _print_stats_status(state='printing', print_duration=100.0):
    return {'state': state, 'print_duration': print_duration}


def _virtual_sdcard_status(is_active=True, file_position=1000,
                            file_path='/sdcard/test.gcode'):
    return {'is_active': is_active, 'file_position': file_position,
            'file_path': file_path}


def _file_info(path='test.gcode', size=100, sha256='a' * 64):
    return {'path': path, 'size': size, 'sha256': sha256}


def _full_sidecar_state(generation=1, file_position=1000):
    return plr.build_sidecar_state(
        generation, _file_info(), file_position, _gcode_move_status(),
        _toolhead_status(), {'target': 200.0, 'bed_target': 60.0,
                              'pressure_advance': 0.04, 'smooth_time': 0.04},
        {'profile_name': 'default'},
        {'objects': ['obj1', 'obj2'], 'excluded_objects': ['obj1']},
        {'speed': 1.0})


class MeaningfulProgressTests(unittest.TestCase):
    def test_all_conditions_met(self):
        self.assertTrue(plr.meaningful_progress(
            _print_stats_status(), _virtual_sdcard_status(),
            _toolhead_status(z=1.0), 0.6))

    def test_not_printing(self):
        self.assertFalse(plr.meaningful_progress(
            _print_stats_status(state='standby'), _virtual_sdcard_status(),
            _toolhead_status(z=1.0), 0.6))

    def test_sdcard_not_active(self):
        self.assertFalse(plr.meaningful_progress(
            _print_stats_status(), _virtual_sdcard_status(is_active=False),
            _toolhead_status(z=1.0), 0.6))

    def test_zero_file_position(self):
        self.assertFalse(plr.meaningful_progress(
            _print_stats_status(), _virtual_sdcard_status(file_position=0),
            _toolhead_status(z=1.0), 0.6))

    def test_zero_print_duration(self):
        self.assertFalse(plr.meaningful_progress(
            _print_stats_status(print_duration=0),
            _virtual_sdcard_status(), _toolhead_status(z=1.0), 0.6))

    def test_z_below_threshold(self):
        self.assertFalse(plr.meaningful_progress(
            _print_stats_status(), _virtual_sdcard_status(),
            _toolhead_status(z=0.3), 0.6))

    def test_z_exactly_at_threshold_not_yet_meaningful(self):
        self.assertFalse(plr.meaningful_progress(
            _print_stats_status(), _virtual_sdcard_status(),
            _toolhead_status(z=0.6), 0.6))


class SidecarStateBuildTests(unittest.TestCase):
    def test_schema_fields_present(self):
        state = _full_sidecar_state()
        self.assertEqual(state['schema_version'], plr.SCHEMA_VERSION)
        self.assertEqual(state['generation'], 1)
        self.assertEqual(state['file'], _file_info())
        self.assertEqual(state['gcode']['gcode_position'], [1.0, 2.0, 10.0, 5.0])
        self.assertEqual(state['gcode']['homing_origin'], [0.0, 0.0, 0.5, 0.0])
        self.assertEqual(state['motion']['max_velocity'], 500.0)
        self.assertEqual(state['thermal']['extruder_target'], 200.0)
        self.assertEqual(state['thermal']['bed_target'], 60.0)
        self.assertEqual(state['extruder']['pressure_advance'], 0.04)
        self.assertEqual(state['bed_mesh']['profile_name'], 'default')
        self.assertEqual(state['exclude_object']['objects'], ['obj1', 'obj2'])
        self.assertEqual(state['fan']['speed'], 1.0)

    def test_json_serializable(self):
        state = _full_sidecar_state()
        json.dumps(state)  # must not raise (Coord objects converted to lists)

    def test_no_private_base_position_leak(self):
        state = _full_sidecar_state()
        self.assertNotIn('base_position', state['gcode'])
        blob = json.dumps(state)
        self.assertNotIn('base_position', blob)

    def test_firmware_retraction_optional(self):
        without = _full_sidecar_state()
        self.assertNotIn('firmware_retraction', without)
        state = plr.build_sidecar_state(
            1, _file_info(), 0, _gcode_move_status(), _toolhead_status(),
            {}, {}, {}, {}, firmware_retraction_status={
                'retract_length': 0.6, 'retract_speed': 35,
                'unretract_extra_length': 0, 'unretract_speed': 30})
        self.assertEqual(state['firmware_retraction']['retract_length'], 0.6)


class SidecarFileTransactionTests(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_write_then_read_round_trip(self):
        final = os.path.join(self.tmpdir, 'state-a.json')
        tmp = final + '.tmp'
        state = _full_sidecar_state()
        plr.atomic_write_json(tmp, final, state)
        self.assertFalse(os.path.exists(tmp))
        self.assertTrue(os.path.exists(final))
        loaded = plr.read_sidecar(final)
        self.assertEqual(loaded, state)

    def test_missing_sidecar_returns_none(self):
        self.assertIsNone(plr.read_sidecar(
            os.path.join(self.tmpdir, 'does-not-exist.json')))

    def test_torn_sidecar_returns_none(self):
        final = os.path.join(self.tmpdir, 'state-a.json')
        with open(final, 'wb') as handle:
            handle.write(b'{"schema_version": 1, "generat')  # truncated
        self.assertIsNone(plr.read_sidecar(final))

    def test_replace_is_atomic_no_stale_tmp_left(self):
        final = os.path.join(self.tmpdir, 'state-b.json')
        tmp = final + '.tmp'
        plr.atomic_write_json(tmp, final, {'a': 1})
        plr.atomic_write_json(tmp, final, {'a': 2})
        self.assertFalse(os.path.exists(tmp))
        self.assertEqual(plr.read_sidecar(final), {'a': 2})

    def test_sidecar_path_for_generation_alternates(self):
        self.assertTrue(plr.sidecar_path_for_generation('/x', 2)
                         .endswith('state-a.json'))
        self.assertTrue(plr.sidecar_path_for_generation('/x', 3)
                         .endswith('state-b.json'))

    def test_never_overwrite_sidecar_of_previous_generation(self):
        # Two successive checkpoints (generations 1 and 2) must land on
        # DIFFERENT files - the previous generation's sidecar (still the
        # last-known-good one until the new EEPROM record is committed)
        # is never touched by the new write.
        final_gen1 = plr.sidecar_path_for_generation(self.tmpdir, 1)
        final_gen2 = plr.sidecar_path_for_generation(self.tmpdir, 2)
        self.assertNotEqual(final_gen1, final_gen2)
        plr.atomic_write_json(final_gen1 + '.tmp', final_gen1, {'gen': 1})
        plr.atomic_write_json(final_gen2 + '.tmp', final_gen2, {'gen': 2})
        self.assertEqual(plr.read_sidecar(final_gen1), {'gen': 1})
        self.assertEqual(plr.read_sidecar(final_gen2), {'gen': 2})


class HashFileTests(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_known_sha256(self):
        path = os.path.join(self.tmpdir, 'f.gcode')
        with open(path, 'wb') as handle:
            handle.write(b'hello world')
        import hashlib
        expected = hashlib.sha256(b'hello world').hexdigest()
        self.assertEqual(plr.hash_file(path), expected)

    def test_changed_file_changes_hash(self):
        path = os.path.join(self.tmpdir, 'f.gcode')
        with open(path, 'wb') as handle:
            handle.write(b'version1')
        h1 = plr.hash_file(path)
        with open(path, 'wb') as handle:
            handle.write(b'version2-different-length')
        h2 = plr.hash_file(path)
        self.assertNotEqual(h1, h2)


class ValidateRecoveryTests(unittest.TestCase):
    def _record(self, generation=1, file_position=1000, tombstone=False):
        flags = journal.FLAG_TOMBSTONE if tombstone else journal.FLAG_VALID
        return journal.Record(1, journal.SCHEMA_VERSION, flags, generation,
                               file_position)

    def test_happy_path_requires_allow_unsafe(self):
        record = self._record()
        sidecar = _full_sidecar_state(generation=1)
        with self.assertRaises(plr.PositionUnsafe):
            plr.validate_recovery(record, sidecar, 'test.gcode', 100,
                                   'a' * 64, allow_unsafe=False)

    def test_happy_path_with_allow_unsafe_succeeds(self):
        record = self._record()
        sidecar = _full_sidecar_state(generation=1)
        result = plr.validate_recovery(record, sidecar, 'test.gcode', 100,
                                        'a' * 64, allow_unsafe=True)
        self.assertIs(result, sidecar)

    def test_no_eeprom_record(self):
        with self.assertRaises(plr.IntegrityFailure):
            plr.validate_recovery(None, _full_sidecar_state(), 'test.gcode',
                                   100, 'a' * 64, allow_unsafe=True)

    def test_missing_sidecar(self):
        with self.assertRaises(plr.IntegrityFailure):
            plr.validate_recovery(self._record(), None, 'test.gcode', 100,
                                   'a' * 64, allow_unsafe=True)

    def test_schema_mismatch(self):
        sidecar = dict(_full_sidecar_state())
        sidecar['schema_version'] = 999
        with self.assertRaises(plr.IntegrityFailure):
            plr.validate_recovery(self._record(), sidecar, 'test.gcode',
                                   100, 'a' * 64, allow_unsafe=True)

    def test_stale_sidecar_generation(self):
        record = self._record(generation=5)
        sidecar = _full_sidecar_state(generation=4)  # doesn't match record
        with self.assertRaises(plr.IntegrityFailure):
            plr.validate_recovery(record, sidecar, 'test.gcode', 100,
                                   'a' * 64, allow_unsafe=True)

    def test_missing_gcode_path(self):
        sidecar = dict(_full_sidecar_state())
        sidecar['file'] = dict(sidecar['file'])
        sidecar['file']['path'] = ''
        with self.assertRaises(plr.IntegrityFailure):
            plr.validate_recovery(self._record(), sidecar, None, None, None,
                                   allow_unsafe=True)

    def test_size_mismatch(self):
        with self.assertRaises(plr.IntegrityFailure):
            plr.validate_recovery(self._record(), _full_sidecar_state(),
                                   'test.gcode', 999, 'a' * 64,
                                   allow_unsafe=True)

    def test_sha256_mismatch(self):
        with self.assertRaises(plr.IntegrityFailure):
            plr.validate_recovery(self._record(), _full_sidecar_state(),
                                   'test.gcode', 100, 'b' * 64,
                                   allow_unsafe=True)

    def test_path_mismatch_changed_file_during_print(self):
        with self.assertRaises(plr.IntegrityFailure):
            plr.validate_recovery(self._record(), _full_sidecar_state(),
                                   'different.gcode', 100, 'a' * 64,
                                   allow_unsafe=True)

    def test_allow_unsafe_never_overrides_integrity_failure(self):
        # A file-identity mismatch must stay refused even with
        # allow_unsafe=True - allow_unsafe only ever affects the
        # position-safety gate, checked strictly after every integrity
        # check above it.
        with self.assertRaises(plr.IntegrityFailure):
            plr.validate_recovery(self._record(), _full_sidecar_state(),
                                   'wrong.gcode', 100, 'a' * 64,
                                   allow_unsafe=True)

    def test_tombstoned_record_treated_as_no_recovery_by_caller(self):
        # validate_recovery() itself is only ever called with the result
        # of journal.read_recovery_state(), which already returns None for
        # a tombstoned journal - this test documents that contract
        # explicitly at this module's own boundary.
        record = journal.read_recovery_state(_eeprom_with_tombstone())
        self.assertIsNone(record)


def _eeprom_with_tombstone():
    eeprom = io.BytesIO(bytes([0xFF]) * journal.EEPROM_TOTAL_SIZE)
    journal.commit_checkpoint(eeprom, 100)
    journal.commit_tombstone(eeprom)
    return eeprom


class ResumeGcodeReconstructionTests(unittest.TestCase):
    def test_standard_commands_present_in_order(self):
        state = _full_sidecar_state(file_position=5000)
        lines = plr.build_resume_gcode_lines(state)
        joined = "\n".join(lines)
        self.assertTrue(lines[0].startswith("M23 "))
        self.assertIn("M26 S5000", joined)
        self.assertIn("G90", joined)
        self.assertIn("M82", joined)
        self.assertIn("M220 S100", joined)
        self.assertIn("M221 S100", joined)
        self.assertIn("SET_GCODE_OFFSET Z=0.5 MOVE=0", joined)
        self.assertIn("G92 E5", joined)
        self.assertIn("SET_VELOCITY_LIMIT", joined)
        self.assertIn("SET_PRESSURE_ADVANCE ADVANCE=0.04 SMOOTH_TIME=0.04", joined)
        self.assertIn("BED_MESH_PROFILE LOAD=default", joined)
        self.assertIn("M104 S200", joined)
        self.assertIn("M140 S60", joined)
        # M23 must come before M26, and both before the SET_* restoration.
        self.assertLess(joined.index("M23 "), joined.index("M26 "))

    def test_never_emits_m24_or_motion(self):
        lines = plr.build_resume_gcode_lines(_full_sidecar_state())
        for line in lines:
            self.assertFalse(line.strip().upper().startswith("M24"))
            self.assertFalse(line.strip().upper().startswith("G28"))
            self.assertFalse(line.strip().upper().startswith("G1 "))
            self.assertFalse(line.strip().upper().startswith("G0 "))

    def test_exclude_object_restoration(self):
        state = _full_sidecar_state()
        lines = plr.build_resume_gcode_lines(state)
        self.assertIn("EXCLUDE_OBJECT_DEFINE NAME=obj1", lines)
        self.assertIn("EXCLUDE_OBJECT_DEFINE NAME=obj2", lines)
        self.assertIn("EXCLUDE_OBJECT NAME=obj1", lines)
        self.assertNotIn("EXCLUDE_OBJECT NAME=obj2", lines)

    def test_bed_mesh_clear_when_no_profile(self):
        state = plr.build_sidecar_state(
            1, _file_info(), 0, _gcode_move_status(), _toolhead_status(),
            {}, {'profile_name': ''}, {}, {})
        lines = plr.build_resume_gcode_lines(state)
        self.assertIn("BED_MESH_CLEAR", lines)
        self.assertNotIn("BED_MESH_PROFILE LOAD=", "\n".join(lines))

    def test_relative_extrude_mode_uses_m83_g91(self):
        state = plr.build_sidecar_state(
            1, _file_info(), 0,
            _gcode_move_status(abs_coord=False, abs_extrude=False),
            _toolhead_status(), {}, {}, {}, {})
        lines = plr.build_resume_gcode_lines(state)
        self.assertIn("G91", lines)
        self.assertIn("M83", lines)
        self.assertNotIn("G90", lines)
        self.assertNotIn("M82", lines)

    def test_fan_off_emits_m107(self):
        state = plr.build_sidecar_state(
            1, _file_info(), 0, _gcode_move_status(), _toolhead_status(),
            {}, {}, {}, {'speed': 0.0})
        lines = plr.build_resume_gcode_lines(state)
        self.assertIn("M107", lines)

    def test_no_heater_commands_when_targets_zero(self):
        state = plr.build_sidecar_state(
            1, _file_info(), 0, _gcode_move_status(), _toolhead_status(),
            {'target': 0.0, 'bed_target': 0.0}, {}, {}, {})
        lines = plr.build_resume_gcode_lines(state)
        self.assertFalse(any(l.startswith("M104") for l in lines))
        self.assertFalse(any(l.startswith("M140") for l in lines))

    def test_retraction_restored_when_configured(self):
        state = plr.build_sidecar_state(
            1, _file_info(), 0, _gcode_move_status(), _toolhead_status(),
            {}, {}, {}, {}, firmware_retraction_status={
                'retract_length': 0.6, 'retract_speed': 35,
                'unretract_extra_length': 0, 'unretract_speed': 30})
        lines = plr.build_resume_gcode_lines(state)
        self.assertTrue(any(l.startswith("SET_RETRACTION") for l in lines))


# ---------------------------------------------------------------------------
# Glue-level tests: exercise the real NebulaOSPowerLossRecovery class end to
# end, faking only the two genuine boundaries (the printer object graph, and
# choosing real tmp files for the EEPROM/sidecar paths so the exact same
# open()/seek()/read()/write() code path used on hardware is exercised).
# ---------------------------------------------------------------------------

class _FakeGCode(object):
    def __init__(self):
        self.commands = {}
        self.run_lines = []
        self.messages = []

    def register_command(self, name, func, desc=None, when_not_ready=False):
        self.commands[name] = func

    def run_script_from_command(self, line):
        self.run_lines.append(line)

    def respond_info(self, msg, log=True):
        self.messages.append(msg)

    def respond_raw(self, msg):
        self.messages.append(msg)


class _FakeGCmd(object):
    error = Exception

    def __init__(self, params=None):
        self._params = params or {}
        self.messages = []

    def get_int(self, name, default, minval=None, maxval=None):
        return int(self._params.get(name, default))

    def respond_info(self, msg, log=True):
        self.messages.append(msg)


class _FakeExecutor(object):
    """Runs the submitted function synchronously, in-process - the real
    aio_executor.Executor.submit() blocks the calling greenlet until the
    background thread finishes anyway, so a synchronous stand-in preserves
    the exact same observable ordering these tests care about."""
    def submit(self, fn, *args, **kwargs):
        return fn(*args, **kwargs)


class _FakeAioDispatcher(object):
    def allocate_executor(self, name=""):
        return _FakeExecutor()


class _FakeReactor(object):
    NOW = 0.
    NEVER = 9999999999999999.

    def __init__(self):
        self.timers = []

    def register_timer(self, callback, waketime=NEVER):
        self.timers.append(callback)
        return callback

    def monotonic(self):
        return 0.0


class _FakePrinter(object):
    def __init__(self, objects):
        self._objects = objects
        self._reactor = _FakeReactor()
        self.event_handlers = {}
        self.config_error = Exception

    def get_reactor(self):
        return self._reactor

    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)

    def send_event(self, event, *params):
        for cb in self.event_handlers.get(event, []):
            cb(*params)

    _sentinel = object()

    def lookup_object(self, name, default=_sentinel):
        if name in self._objects:
            return self._objects[name]
        if default is self._sentinel:
            raise Exception("unknown fake printer object %r" % (name,))
        return default

    def load_object(self, config, section, default=None):
        return self.lookup_object(section, default)


class _FakeConfig(object):
    def __init__(self, printer, values=None):
        self._printer = printer
        self._values = values or {}

    def get_printer(self):
        return self._printer

    def get(self, option, default=None):
        return self._values.get(option, default)

    def getfloat(self, option, default=None, above=None, minval=None):
        return float(self._values.get(option, default))


class _FakeStatusObject(object):
    def __init__(self, status):
        self._status = status

    def get_status(self, eventtime=None):
        return self._status


class _FakeMcu(object):
    """estimated_print_time() is the real-time side of the physical-
    completion proof (see the module's own "Checkpoint execution
    semantics" comment) - tests drive it directly rather than simulating
    an actual MCU clock."""
    def __init__(self):
        self.current_estimated_print_time = 0.0

    def estimated_print_time(self, eventtime=None):
        return self.current_estimated_print_time


class _FakeToolhead(_FakeStatusObject):
    """register_lookahead_callback() here does NOT fire immediately (a
    real, empty-queue Klipper toolhead would) - tests fire callbacks
    explicitly via fire_pending_callbacks() so the two-stage candidate ->
    durable timing can be driven precisely and deterministically."""
    def __init__(self, status):
        super(_FakeToolhead, self).__init__(status)
        self.pending_callbacks = []

    def register_lookahead_callback(self, callback):
        self.pending_callbacks.append(callback)

    def fire_pending_callbacks(self, print_time):
        callbacks, self.pending_callbacks = self.pending_callbacks, []
        for callback in callbacks:
            callback(print_time)


class ExtensionStateMachineTests(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.eeprom_path = os.path.join(self.tmpdir, 'eeprom')
        with open(self.eeprom_path, 'wb') as handle:
            handle.write(bytes([0xFF]) * journal.EEPROM_TOTAL_SIZE)
        self.sidecar_dir = os.path.join(self.tmpdir, 'plr')

        gcode_move_status = _gcode_move_status()
        toolhead_status = _toolhead_status()
        self.print_stats_status = _print_stats_status()
        self.vsd_status = _virtual_sdcard_status(
            file_path=os.path.join(self.tmpdir, 'print.gcode'))
        with open(self.vsd_status['file_path'], 'wb') as handle:
            handle.write(b'; test gcode file contents')

        self.objects = {
            'gcode': _FakeGCode(),
            'gcode_move': _FakeStatusObject(gcode_move_status),
            'toolhead': _FakeToolhead(toolhead_status),
            'mcu': _FakeMcu(),
            'print_stats': _FakeStatusObject(self.print_stats_status),
            'virtual_sdcard': type('VSD', (_FakeStatusObject,), {
                'sdcard_dirname': self.tmpdir})(self.vsd_status),
            'exclude_object': _FakeStatusObject(
                {'objects': [], 'excluded_objects': []}),
            'bed_mesh': _FakeStatusObject({'profile_name': ''}),
            'extruder': _FakeStatusObject({'target': 0.0,
                                           'pressure_advance': 0.04,
                                           'smooth_time': 0.04}),
            'heater_bed': _FakeStatusObject({'target': 0.0}),
            'fan': _FakeStatusObject({'speed': 0.0}),
            'aio_executor': _FakeAioDispatcher(),
        }
        self.printer = _FakePrinter(self.objects)
        self.config = _FakeConfig(self.printer, {
            'eeprom_path': self.eeprom_path,
            'sidecar_dir': self.sidecar_dir,
            'checkpoint_interval': 5.0,
            'poll_interval': 0.75,
            'min_z_for_start': 0.6,
        })
        self.ext = plr.NebulaOSPowerLossRecovery(self.config)
        self.ext._handle_ready()

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _eeprom_record(self):
        with open(self.eeprom_path, 'r+b') as handle:
            return journal.read_recovery_state(handle)

    def _eeprom_scan(self):
        # Unlike _eeprom_record()/read_recovery_state(), this returns the
        # newest record even if it's a tombstone - for asserting "a
        # tombstone was actually written" rather than just "no recovery is
        # available" (which is also true of e.g. total EEPROM corruption).
        with open(self.eeprom_path, 'r+b') as handle:
            return journal.scan_journal(handle)

    def _tick_and_settle(self, eventtime):
        """Ticks once (which may take a new Candidate), then simulates
        "motion completes instantly" (fire any pending toolhead timing
        callback with a low print_time, advance the fake mcu's estimated
        real time far ahead of it) and ticks again so any candidate taken
        this round is promoted to durable immediately. For tests that
        aren't specifically exercising the two-stage candidate timing
        itself - see CandidatePromotionTests for those."""
        self.ext._tick(eventtime)
        self.objects['toolhead'].fire_pending_callbacks(0.0)
        self.objects['mcu'].current_estimated_print_time = 1e9
        self.ext._tick(eventtime)

    def test_meaningful_progress_gate_blocks_session_start(self):
        self.objects['toolhead']._status = _toolhead_status(z=0.1)
        self._tick_and_settle(1.0)
        self.assertFalse(self.ext._have_active_session)
        self.assertIsNone(self._eeprom_record())

    def test_session_starts_and_first_checkpoint_commits(self):
        self._tick_and_settle(1.0)
        self.assertTrue(self.ext._have_active_session)
        record = self._eeprom_record()
        self.assertIsNotNone(record)
        self.assertEqual(record.file_position,
                          self.vsd_status['file_position'])

    def test_checkpoint_5s_rate_limit(self):
        self._tick_and_settle(1.0)
        first = self._eeprom_record()
        self.vsd_status['file_position'] = 2000
        self._tick_and_settle(1.5)  # well under 5s later - must NOT checkpoint again
        second = self._eeprom_record()
        self.assertEqual(first.generation, second.generation)
        self._tick_and_settle(6.5)  # now past the 5s interval
        third = self._eeprom_record()
        self.assertGreater(third.generation, second.generation)
        self.assertEqual(third.file_position, 2000)

    def test_checkpoint_in_flight_guard_skips_rather_than_queues(self):
        self._tick_and_settle(1.0)
        self.ext._checkpoint_in_flight = True
        before = self._eeprom_record()
        self._tick_and_settle(10.0)  # due for a checkpoint, but marked in-flight
        after = self._eeprom_record()
        self.assertEqual(before.generation, after.generation)

    def test_pause_preserves_checkpoint_no_new_writes(self):
        self._tick_and_settle(1.0)
        before = self._eeprom_record()
        self.print_stats_status['state'] = 'paused'
        self._tick_and_settle(10.0)
        after = self._eeprom_record()
        self.assertEqual(before.generation, after.generation)
        self.assertTrue(self.ext._have_active_session)

    def test_resume_after_pause_continues_checkpointing(self):
        self._tick_and_settle(1.0)
        self.print_stats_status['state'] = 'paused'
        self._tick_and_settle(2.0)
        self.print_stats_status['state'] = 'printing'
        self.vsd_status['file_position'] = 3000
        self._tick_and_settle(10.0)
        record = self._eeprom_record()
        self.assertEqual(record.file_position, 3000)

    def test_complete_triggers_immediate_tombstone(self):
        self._tick_and_settle(1.0)
        self.assertTrue(self.ext._have_active_session)
        self.print_stats_status['state'] = 'complete'
        self._tick_and_settle(2.0)
        self.assertFalse(self.ext._have_active_session)
        self.assertIsNone(self._eeprom_record())

    def test_cancelled_triggers_immediate_tombstone(self):
        self._tick_and_settle(1.0)
        self.print_stats_status['state'] = 'cancelled'
        self._tick_and_settle(2.0)
        self.assertIsNone(self._eeprom_record())

    def test_error_preserves_checkpoint_no_tombstone(self):
        self._tick_and_settle(1.0)
        before = self._eeprom_record()
        self.print_stats_status['state'] = 'error'
        self._tick_and_settle(2.0)
        after = self._eeprom_record()
        self.assertEqual(before.generation, after.generation)
        self.assertFalse(after.is_tombstone)

    def test_virtual_sdcard_reset_during_active_session_invalidates_plr(self):
        self._tick_and_settle(1.0)
        self.assertTrue(self.ext._have_active_session)
        self.printer.send_event("virtual_sdcard:reset_file")
        self.assertFalse(self.ext._have_active_session)
        self.assertIsNone(self._eeprom_record())

    def test_resume_in_progress_guard_ignores_own_reset_file(self):
        self._tick_and_settle(1.0)
        self.assertTrue(self.ext._have_active_session)
        self.ext._resume_in_progress = True
        self.printer.send_event("virtual_sdcard:reset_file")
        # Session must survive - this is our OWN M23 from a resume flow,
        # not a foreign file change.
        self.assertTrue(self.ext._have_active_session)
        self.assertIsNotNone(self._eeprom_record())

    def test_manual_discard_tombstones(self):
        self._tick_and_settle(1.0)
        gcmd = _FakeGCmd()
        self.ext.cmd_NEBULAOS_PLR_DISCARD(gcmd)
        self.assertIsNone(self._eeprom_record())
        self.assertFalse(self.ext._have_active_session)

    def test_resume_refuses_without_allow_unsafe(self):
        self._tick_and_settle(1.0)
        gcmd = _FakeGCmd({'ALLOW_UNSAFE': 0})
        with self.assertRaises(Exception):
            self.ext.cmd_NEBULAOS_PLR_RESUME(gcmd)
        self.assertEqual(self.objects['gcode'].run_lines, [])

    def test_resume_one_time_allow_unsafe_succeeds_and_restores_state(self):
        self._tick_and_settle(1.0)
        gcmd = _FakeGCmd({'ALLOW_UNSAFE': 1})
        self.ext.cmd_NEBULAOS_PLR_RESUME(gcmd)
        lines = self.objects['gcode'].run_lines
        self.assertTrue(any(l.startswith("M23 ") for l in lines))
        self.assertTrue(any(l.startswith("M26 ") for l in lines))
        self.assertFalse(self.ext._resume_in_progress)  # cleared afterward

    def test_resume_allow_unsafe_cannot_bypass_integrity_failure(self):
        self._tick_and_settle(1.0)
        # Corrupt the on-disk gcode file so its hash no longer matches what
        # was recorded at session start.
        with open(self.vsd_status['file_path'], 'ab') as handle:
            handle.write(b'; tampered after checkpoint')
        gcmd = _FakeGCmd({'ALLOW_UNSAFE': 1})
        with self.assertRaises(Exception):
            self.ext.cmd_NEBULAOS_PLR_RESUME(gcmd)
        self.assertEqual(self.objects['gcode'].run_lines, [])

    def test_status_command_reports_available_recovery(self):
        self._tick_and_settle(1.0)
        gcmd = _FakeGCmd()
        self.ext.cmd_NEBULAOS_PLR_STATUS(gcmd)
        self.assertTrue(any('recovery available' in m for m in gcmd.messages))

    def test_status_command_reports_none_when_empty(self):
        gcmd = _FakeGCmd()
        self.ext.cmd_NEBULAOS_PLR_STATUS(gcmd)
        self.assertTrue(any('no recovery available' in m for m in gcmd.messages))


class CandidatePromotionTests(ExtensionStateMachineTests):
    """The two-stage candidate -> durable pipeline (see the module's own
    "Checkpoint execution semantics" comment): file_position proves gcode
    was DISPATCHED, not physically executed. These tests drive the fake
    toolhead's register_lookahead_callback()/the fake mcu's
    estimated_print_time() directly, deliberately WITHOUT the
    _tick_and_settle() convenience helper the other test class uses, so
    each stage of the real mechanism is exercised explicitly."""

    def test_file_position_ahead_of_physical_queue_not_yet_durable(self):
        # A candidate is taken (file_position snapshotted), but its
        # associated motion has not been reported complete at all yet -
        # nothing may be written to the EEPROM/sidecar.
        self.ext._tick(1.0)
        self.assertIsNotNone(self.ext._pending_candidate)
        self.assertIsNone(self._eeprom_record())

    def test_candidate_not_promoted_while_timing_unknown(self):
        self.ext._tick(1.0)
        # Real time "catches up" to an arbitrarily high print_time, but
        # the toolhead has not yet confirmed ANY target_print_time for
        # this candidate (its move sits unflushed) - still not durable.
        self.objects['mcu'].current_estimated_print_time = 1e9
        self.ext._tick(1.5)
        self.assertIsNone(self._eeprom_record())

    def test_candidate_not_promoted_while_motion_still_scheduled(self):
        self.ext._tick(1.0)
        self.objects['toolhead'].fire_pending_callbacks(500.0)
        # Real time has NOT yet caught up to the scheduled print_time -
        # the motion is queued but not physically finished.
        self.objects['mcu'].current_estimated_print_time = 100.0
        self.ext._tick(1.5)
        self.assertIsNone(self._eeprom_record())

    def test_motion_completion_promotes_candidate(self):
        self.ext._tick(1.0)
        self.objects['toolhead'].fire_pending_callbacks(500.0)
        self.objects['mcu'].current_estimated_print_time = 500.0  # exactly caught up
        self.ext._tick(1.5)
        record = self._eeprom_record()
        self.assertIsNotNone(record)
        self.assertEqual(record.file_position, self.vsd_status['file_position'])
        self.assertIsNone(self.ext._pending_candidate)

    def test_power_loss_before_promotion_leaves_previous_durable_generation(self):
        self._tick_and_settle(1.0)
        first = self._eeprom_record()
        self.assertIsNotNone(first)

        # A new candidate is taken (more progress), but power is "lost"
        # before its motion is ever reported complete - simulated simply
        # by never firing its callback / never advancing mcu time, i.e.
        # never calling anything that would promote it.
        self.vsd_status['file_position'] = 9999
        self.ext._tick(6.5)
        self.assertIsNotNone(self.ext._pending_candidate)

        # The EEPROM must still show the PREVIOUS durable generation,
        # untouched - a pending candidate is never trusted.
        after = self._eeprom_record()
        self.assertEqual(after.generation, first.generation)
        self.assertEqual(after.file_position, first.file_position)

    def test_superseded_candidate_uses_newest_file_position(self):
        self.ext._tick(1.0)
        stale_position = self.vsd_status['file_position']
        self.assertEqual(self.ext._pending_candidate.file_position, stale_position)

        # The stale candidate's motion never gets reported complete, but
        # real progress keeps happening in the gcode stream and a full
        # checkpoint_interval elapses - the candidate must be superseded
        # by a fresh one at the newer file_position, not persisted stale
        # forever once it does eventually complete.
        self.vsd_status['file_position'] = stale_position + 500
        self.ext._tick(6.5)
        self.assertEqual(self.ext._pending_candidate.file_position,
                          stale_position + 500)

        self.objects['toolhead'].fire_pending_callbacks(0.0)
        self.objects['mcu'].current_estimated_print_time = 1e9
        self.ext._tick(7.0)
        record = self._eeprom_record()
        self.assertEqual(record.file_position, stale_position + 500)

    def test_pause_during_pending_candidate_still_promotable_later(self):
        self.ext._tick(1.0)
        self.assertIsNotNone(self.ext._pending_candidate)

        self.print_stats_status['state'] = 'paused'
        self.objects['toolhead'].fire_pending_callbacks(0.0)
        self.objects['mcu'].current_estimated_print_time = 1e9
        self.ext._tick(1.5)  # promotion check runs regardless of print state

        record = self._eeprom_record()
        self.assertIsNotNone(record)
        self.assertIsNone(self.ext._pending_candidate)

    def test_cancel_during_pending_candidate_discards_it(self):
        self.ext._tick(1.0)
        candidate = self.ext._pending_candidate
        self.assertIsNotNone(candidate)

        self.print_stats_status['state'] = 'cancelled'
        self.ext._tick(1.5)
        self.assertIsNone(self.ext._pending_candidate)

        # Even if the discarded candidate's own callback fires later (a
        # genuine race in real Klipper - the trapq flush doesn't know or
        # care that PLR gave up on it), it must never resurrect a
        # commit - there is no pending candidate left to promote.
        self.objects['toolhead'].fire_pending_callbacks(0.0)
        self.objects['mcu'].current_estimated_print_time = 1e9
        self.ext._tick(2.0)
        record = self._eeprom_scan()
        self.assertIsNotNone(record)
        self.assertTrue(record.is_tombstone)

    def test_complete_during_pending_candidate_discards_it(self):
        self.ext._tick(1.0)
        self.assertIsNotNone(self.ext._pending_candidate)

        self.print_stats_status['state'] = 'complete'
        self.ext._tick(1.5)
        self.assertIsNone(self.ext._pending_candidate)
        record = self._eeprom_scan()
        self.assertIsNotNone(record)
        self.assertTrue(record.is_tombstone)

    def test_no_duplicate_or_skipped_generation_ordering(self):
        seen_generations = []
        eventtime = 1.0
        for i in range(4):
            self.vsd_status['file_position'] = 1000 + i * 100
            self._tick_and_settle(eventtime)
            record = self._eeprom_record()
            seen_generations.append(record.generation)
            eventtime += self.ext.checkpoint_interval + 0.5
        self.assertEqual(seen_generations,
                          list(range(seen_generations[0],
                                     seen_generations[0] + 4)))


if __name__ == '__main__':
    unittest.main()

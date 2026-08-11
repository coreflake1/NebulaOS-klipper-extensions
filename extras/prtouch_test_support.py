# Shared, offline test-only fakes for the prtouch_v2/z_compensate module set.
#
# Built to exercise the REAL production code (PrtouchMCU, PrtouchProbe, PRTouchV2,
# ZCompensate) against a deterministic fake MCU/config/printer, not to reimplement or
# mirror that code's own logic. Every fake here models one real Klipper API surface
# (ConfigWrapper, MCU/CommandWrapper, ppins, toolhead, reactor) closely enough that the
# production modules can be instantiated and driven exactly as klippy.py would, with zero
# physical hardware and zero real time elapsed (FakeReactor's clock is a plain float the
# test controls directly - no time.sleep anywhere in this file).
#
# Deliberately NOT a full Klipper reimplementation: only the surface prtouch_v2/z_compensate
# actually calls is modeled. Extend as new call sites are exercised, don't pre-build unused
# surface.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import re


class ConfigError(Exception):
    pass


class CommandError(Exception):
    pass


_SENTINEL = object()


class FakeConfig:
    """Mimics klippy/configfile.py's ConfigWrapper closely enough to reproduce its two
    real, previously-hardware-discovered failure modes:
      1. An option present in the section but never read via some get*() call must be
         detectable as "unused" (Klipper's real check_unused_options) - see
         assert_all_consumed().
      2. minval/maxval/required-without-default must raise, exactly like the real
         configfile.py, so a fixture built from real printer.cfg values will fail a test
         the same way it failed live on 2026-08-06 if a future edit reintroduces the same
         class of bug (an option read with too-strict bounds, or not read at all).

    `values` is a flat dict of this section's own key/value pairs (as literal strings, like
    a real .cfg file - callers should not pre-convert types, that's this class's job,
    matching real Klipper where every value starts as a string from configparser).
    `other_sections` maps section name -> FakeConfig, for config.getsection()/has_section()
    cross-section lookups (prtouch_mcu.py reads the printer's own [printer]/[stepper_z] etc).
    """

    error = ConfigError

    def __init__(self, values, section='test', printer=None, other_sections=None):
        self.section = section
        self._values = dict(values)
        self._accessed = set()
        self.printer = printer
        self._other_sections = other_sections or {}

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.section

    def has_section(self, name):
        return name in self._other_sections

    def getsection(self, name):
        return self._other_sections[name]

    def _raw(self, option, default, note_valid=True):
        if option not in self._values:
            if default is not _SENTINEL:
                return None, default, False
            raise self.error("Option '%s' in section '%s' must be specified"
                              % (option, self.section))
        if note_valid:
            self._accessed.add(option)
        return self._values[option], None, True

    def get(self, option, default=_SENTINEL, note_valid=True):
        raw, dflt, present = self._raw(option, default, note_valid)
        return raw if present else dflt

    def _numeric(self, option, default, minval, maxval, above, below, parser, note_valid):
        raw, dflt, present = self._raw(option, default, note_valid)
        if not present:
            return dflt
        try:
            v = parser(raw)
        except (TypeError, ValueError):
            raise self.error("Unable to parse option '%s' in section '%s'"
                              % (option, self.section))
        if minval is not None and v < minval:
            raise self.error("Option '%s' in section '%s' must have minimum of %s"
                              % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise self.error("Option '%s' in section '%s' must have maximum of %s"
                              % (option, self.section, maxval))
        if above is not None and v <= above:
            raise self.error("Option '%s' in section '%s' must be above %s"
                              % (option, self.section, above))
        if below is not None and v >= below:
            raise self.error("Option '%s' in section '%s' must be below %s"
                              % (option, self.section, below))
        return v

    def getint(self, option, default=_SENTINEL, minval=None, maxval=None, note_valid=True):
        return self._numeric(option, default, minval, maxval, None, None, int, note_valid)

    def getfloat(self, option, default=_SENTINEL, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        return self._numeric(option, default, minval, maxval, above, below, float, note_valid)

    def getboolean(self, option, default=_SENTINEL, note_valid=True):
        raw, dflt, present = self._raw(option, default, note_valid)
        if not present:
            return dflt
        if isinstance(raw, bool):
            return raw
        return str(raw).strip().lower() in ('true', 'yes', 'on', '1')

    def getfloatlist(self, option, default=_SENTINEL, count=None, note_valid=True):
        raw, dflt, present = self._raw(option, default, note_valid)
        if not present:
            return dflt
        if isinstance(raw, (list, tuple)):
            parts = raw
        else:
            parts = [p.strip() for p in str(raw).split(',')]
        values = tuple(float(p) for p in parts)
        if count is not None and len(values) != count:
            raise self.error("Option '%s' in section '%s' must have %d values"
                              % (option, self.section, count))
        return values

    def assert_all_consumed(self):
        """Fails the way a real klippy startup would fail: any key present in the fixture
        that no get*() call ever touched is exactly Klipper's real "Option '...' is not
        valid in section '...'" - confirmed live 2026-08-06 (clr_noz_start_x, deferred to a
        gcode-command body instead of __init__, tripped exactly this)."""
        unused = sorted(set(self._values) - self._accessed)
        if unused:
            raise self.error(
                "Option(s) %s in section '%s' present but never read (would be rejected by "
                "real Klipper at startup)" % (unused, self.section))


class FakeReactor:
    """Virtual clock - pause() advances an in-memory float, never sleeps. schedule_at()
    lets a test (or the harness scenario machinery) queue a callback to fire the next time
    pause() reaches or passes that virtual time, which is how async MCU responses get
    delivered "during" a collect_*_samples poll loop without any real waiting."""

    def __init__(self, start=0.0):
        self._now = start
        self._scheduled = []  # list of [fire_at, callback]

    def monotonic(self):
        return self._now

    def pause(self, waketime):
        self._now = max(self._now, waketime)
        self._fire_due()
        return self._now

    def schedule_at(self, fire_at, callback):
        self._scheduled.append([fire_at, callback])

    def schedule_after(self, delay, callback):
        self.schedule_at(self._now + delay, callback)

    def _fire_due(self):
        while True:
            due = [e for e in self._scheduled if e[0] <= self._now]
            if not due:
                return
            due.sort(key=lambda e: e[0])
            event = due[0]
            self._scheduled.remove(event)
            event[1]()


_FIELD_RE = re.compile(r'(\w+)=%')


def _field_names(fmt):
    """'start_step_prtouch oid=%c dir=%c ...' -> ['oid', 'dir', ...] - used so protocol
    tests can assert on named fields instead of raw positional indices."""
    return _FIELD_RE.findall(fmt)


class _SentCall:
    def __init__(self, name, fields, args):
        self.name = name
        self.args = list(args)
        self.by_field = dict(zip(fields, args)) if len(fields) == len(args) else None


class FakeCommand:
    """lookup_command()-equivalent: fire-and-forget. .send() is recorded and, if the
    scenario registered an on_send hook for this command name, that hook runs synchronously
    (letting a scenario schedule an async response as a direct, traceable consequence of
    this exact send)."""

    def __init__(self, mcu, fmt):
        self.mcu = mcu
        self.name = fmt.split()[0]
        self.fields = _field_names(fmt)

    def send(self, args):
        call = _SentCall(self.name, self.fields, args)
        self.mcu.sent_commands.append(call)
        hook = self.mcu.on_send.get(self.name)
        if hook is not None:
            hook(call)
        return None


class FakeQueryCommand:
    """lookup_query_command()-equivalent: synchronous send-and-get-response. The response
    is whatever the scenario configured via mcu.set_query_response()/queue_query_response()
    for this command name - a fixed dict, a list consumed in order, or a callable(call)."""

    def __init__(self, mcu, fmt, resp_fmt, oid=None):
        self.mcu = mcu
        self.name = fmt.split()[0]
        self.resp_name = resp_fmt.split()[0]
        self.fields = _field_names(fmt)
        self.oid = oid

    def send(self, args):
        call = _SentCall(self.name, self.fields, args)
        self.mcu.sent_commands.append(call)
        provider = self.mcu.query_responses.get(self.name)
        if provider is None:
            raise AssertionError(
                "FakeMCU: no response configured for query command '%s' (call: %s) - "
                "every query send in a test must have a scripted response, silently "
                "returning None would hide a real protocol gap" % (self.name, call.args))
        if callable(provider):
            return provider(call)
        if isinstance(provider, list):
            if not provider:
                raise AssertionError(
                    "FakeMCU: query response queue for '%s' exhausted" % self.name)
            return provider.pop(0)
        return provider


class FakeMCU:
    """The 'chip' object returned by ppins.parse_pin()['chip'] - stands in for both
    step_mcu and pres_mcu. A single instance is normally shared for both (this printer has
    one physical MCU for the whole prtouch protocol), but tests are free to use two
    instances to probe cross-MCU assumptions."""

    def __init__(self, name='mcu'):
        self.name = name
        self._next_oid = 0
        self.config_callbacks = []
        self.config_cmds = []
        self.sent_commands = []
        self.on_send = {}
        self.query_responses = {}
        self.response_handlers = {}  # name -> {oid: handler}

    def create_oid(self):
        oid = self._next_oid
        self._next_oid += 1
        return oid

    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)

    def run_config_callbacks(self):
        for cb in self.config_callbacks:
            cb()

    def add_config_cmd(self, cmd_text):
        self.config_cmds.append(cmd_text)

    def lookup_command(self, fmt, cq=None):
        return FakeCommand(self, fmt)

    def lookup_query_command(self, fmt, resp_fmt, oid=None):
        cmd = FakeQueryCommand(self, fmt, resp_fmt, oid=oid)
        return cmd

    def register_response(self, handler, name, oid):
        self.response_handlers.setdefault(name, {})[oid] = handler

    # -- scenario-facing API ---------------------------------------------------------

    def set_query_response(self, name, response):
        self.query_responses[name] = response

    def on_send_hook(self, name, callback):
        self.on_send[name] = callback

    def push_response(self, name, oid, params):
        """Deliver a simulated async response immediately - call this from inside a
        FakeReactor-scheduled callback (or directly, for an immediate/eager push) to
        exercise the real _handle_result_* methods exactly as Klipper's serial layer
        would invoke them."""
        handler = self.response_handlers.get(name, {}).get(oid)
        if handler is None:
            raise AssertionError(
                "FakeMCU: no response handler registered for '%s' oid=%s" % (name, oid))
        handler(params)

    def last_call(self, name):
        for call in reversed(self.sent_commands):
            if call.name == name:
                return call
        return None

    def all_calls(self, name):
        return [c for c in self.sent_commands if c.name == name]


class FakePins:
    def __init__(self, chip_by_pin):
        # pin name (without prefix chars) -> FakeMCU
        self._chip_by_pin = chip_by_pin

    def parse_pin(self, pin_desc, can_invert=True, can_pullup=True):
        invert = 0
        name = pin_desc
        if can_invert and name.startswith('!'):
            invert = 1
            name = name[1:]
        chip = self._chip_by_pin.get(name)
        if chip is None:
            raise AssertionError("FakePins: no chip configured for pin '%s'" % pin_desc)
        return {'chip': chip, 'pin': name, 'invert': invert}


class FakeStepper:
    def __init__(self, axis='z', step_dist=0.005):
        self._axis = axis
        self._step_dist = step_dist

    def is_active_axis(self, axis):
        return axis == self._axis

    def get_step_dist(self):
        return self._step_dist


class FakeKinematics:
    def __init__(self, steppers):
        self._steppers = steppers

    def get_steppers(self):
        return self._steppers


class FakeToolhead:
    def __init__(self, position=(0., 0., 0., 0.), step_dist=0.005):
        self._position = list(position)
        self._kin = FakeKinematics([FakeStepper('z', step_dist)])
        self.moves = []
        self.homed_axes = 'xyz'  # matches Klipper's real toolhead.get_status() field name/shape

    def get_position(self):
        return list(self._position)

    def set_position_z(self, z):
        self._position[2] = z

    def get_kinematics(self):
        return self._kin

    def wait_moves(self):
        pass

    def get_status(self, eventtime):
        return {'homed_axes': self.homed_axes}


class FakeBedMesh:
    class _Bmc:
        def __init__(self, mesh_min, mesh_max):
            self.mesh_min = mesh_min
            self.mesh_max = mesh_max

    def __init__(self, mesh_min=(5., 10.), mesh_max=(215., 215.)):
        self.bmc = self._Bmc(mesh_min, mesh_max)
        self._mesh = None
        self.set_mesh_calls = []

    def get_mesh(self):
        return self._mesh

    def set_mesh(self, mesh):
        self.set_mesh_calls.append(mesh)
        self._mesh = mesh


class FakeHeater:
    def __init__(self, target_temp=0.0, smoothed_temp=25.0):
        self.target_temp = target_temp
        self.smoothed_temp = smoothed_temp


class FakePHeaters:
    def __init__(self):
        self.calls = []

    def set_temperature(self, heater, temp, wait):
        self.calls.append((heater, temp, wait))
        heater.target_temp = temp
        heater.smoothed_temp = temp  # instant settle - offline test, no thermal simulation


class FakeHeaterBed:
    def __init__(self):
        self.heater = FakeHeater()

    def get_status(self, eventtime):
        return {'target': self.heater.target_temp}


class FakeExtruder:
    def __init__(self):
        self.heater = FakeHeater()


class FakeBLTouchProbe:
    """Stands in for Klipper's real `probe` object (bltouch.py) - z_compensate.py's
    _handle_connect resolves this via lookup_object('probe') but never calls into it
    directly today (bl_offset is read straight from [z_compensate]'s own config, not from
    this object) - present so the connect sequence matches production exactly."""

    def __init__(self, z_offset=0.0):
        self.z_offset = z_offset


class FakeGCode:
    """Records every gcode script it's asked to run instead of executing anything -
    scripts are inert here by construction (see movement_guard.py for the belt-and-braces
    version used against the real gcode object)."""

    def __init__(self):
        self.commands = {}
        self.scripts_run = []

    def register_command(self, name, handler, desc=None):
        self.commands[name] = handler

    def run_script_from_command(self, script):
        self.scripts_run.append(script)

    def respond_info(self, msg):
        pass


class FakeGCmd:
    def __init__(self, params=None):
        self._params = params or {}
        self.responses = []

    def _checked(self, name, value, minval=None, maxval=None, above=None, below=None):
        # mirrors the real GCodeCommand's own bound-checking - a fake that silently ignored
        # these would make maxval=/above= constraints on real commands untestable here.
        if value is None:
            return value
        if minval is not None and value < minval:
            raise self.error("%s must be at least %s" % (name, minval))
        if maxval is not None and value > maxval:
            raise self.error("%s must be at most %s" % (name, maxval))
        if above is not None and value <= above:
            raise self.error("%s must be above %s" % (name, above))
        if below is not None and value >= below:
            raise self.error("%s must be below %s" % (name, below))
        return value

    def get_float(self, name, default=None, **kwargs):
        value = float(self._params[name]) if name in self._params else default
        return self._checked(name, value, **kwargs)

    def get_int(self, name, default=None, **kwargs):
        value = int(self._params[name]) if name in self._params else default
        return self._checked(name, value, **kwargs)

    def respond_info(self, msg):
        self.responses.append(msg)

    def error(self, msg):
        return CommandError(msg)


class FakePrinter:
    error = ConfigError

    def __init__(self):
        self.reactor = FakeReactor()
        self.objects = {}
        self.event_handlers = {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=_SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is not _SENTINEL:
            return default
        raise ConfigError("Unknown config object '%s'" % name)

    def add_object(self, name, obj):
        self.objects[name] = obj

    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)

    def send_event(self, event, *args):
        for cb in self.event_handlers.get(event, []):
            cb(*args)

    def config_error(self, msg):
        return ConfigError(msg)

    def command_error(self, msg):
        return CommandError(msg)


def make_step_result(step_oid, tri_time_ticks, samples):
    """Build the 4 params dicts (result_run_step_prtouch's own real chunk size) needed to
    push `samples` (a list of (tick_ticks, step) pairs, len multiple of 4) through
    FakeMCU.push_response for 'result_run_step_prtouch'."""
    assert len(samples) % 4 == 0
    chunks = []
    for base in range(0, len(samples), 4):
        group = samples[base:base + 4]
        params = {'oid': step_oid, 'tri_time': tri_time_ticks, 'index': base}
        for j, (tick, step) in enumerate(group):
            params['tick%d' % j] = tick
            params['step%d' % j] = step
        chunks.append(params)
    return chunks


#: This printer's own real [prtouch_v2]/[z_compensate] values, pulled live via SSH
#: 2026-08-06 and cross-checked against factory_printer.cfg (see printer.cfg's own
#: [prtouch_v2]/[z_compensate] sections) - used as the default fixture so config-parity
#: tests exercise the actual real-world values, not synthetic ones that might happen to
#: avoid a bound a real value would hit.
REAL_PRTOUCH_V2_CONFIG = {
    'pres_cnt': '1', 'pres0_clk_pins': 'PA4', 'pres0_sdo_pins': 'PC6',
    'step_swap_pin': 'PA15', 'pres_swap_pin': 'PA15', 'step_base': '2',
    'tri_min_hold': '1000', 'tri_max_hold': '1500', 'speed': '1',
}

REAL_Z_COMPENSATE_CONFIG = {
    'tri_min_hold': '1400', 'tri_max_hold': '2000', 'tri_expand_mm': '0.10',
    'speed': '5', 'hot_start_temp': '180', 'hot_rub_temp': '200', 'hot_end_temp': '140',
    'bed_add_temp': '60', 'clr_noz_start_x': '-3', 'clr_noz_start_y': '20',
    'clr_noz_len_x': '3', 'clr_noz_len_y': '50', 'pa_clr_dis_mm_x': '0',
    'pa_clr_dis_mm_y': '30', 'bl_offset': '0, 27', 'noz_pos_center': '20, 25',
    'noz_pos_offset': '3, 7', 'pumpback_mm': '10', 'vs_start_z_pos': '3',
    'pr_probe_cnt': '3', 'pr_clear_probe_cnt': '3', 'type_nozz': '0',
}

REAL_BLTOUCH_Y_OFFSET = 27.  # printer.cfg [bltouch] y_offset - bl_offset must match this


def build_environment(prtouch_v2_values=None, mesh_min=(5., 10.), mesh_max=(215., 215.)):
    """Assembles the fake printer/pins/toolhead/heaters graph PRTouchV2.__init__ and its
    klippy:connect handler actually need, mirroring real printer.cfg's hardware section
    (shared PA15 for both swap pins, PB6/PB5 for stepper_z, matches this printer's real
    wiring) closely enough that pin resolution exercises the same code path as production.
    Returns (printer, mcu, config_dict) - config_dict is the raw values dict, still mutable
    by the caller before it's handed to a FakeConfig.
    """
    printer = FakePrinter()
    mcu = FakeMCU()
    pins = FakePins({
        'PA15': mcu, 'PB6': mcu, 'PB5': mcu, 'PA4': mcu, 'PC6': mcu,
    })
    printer.add_object('pins', pins)
    printer.add_object('gcode', FakeGCode())
    printer.add_object('toolhead', FakeToolhead(step_dist=0.005))
    printer.add_object('bed_mesh', FakeBedMesh(mesh_min, mesh_max))
    printer.add_object('probe', FakeBLTouchProbe())
    printer.add_object('heater_bed', FakeHeaterBed())
    printer.add_object('extruder', FakeExtruder())
    printer.add_object('heaters', FakePHeaters())
    values = dict(REAL_PRTOUCH_V2_CONFIG)
    if prtouch_v2_values:
        values.update(prtouch_v2_values)
    return printer, mcu, pins, values


def make_prtouch_v2_config(printer, pins, values, section='prtouch_v2'):
    printer_section = FakeConfig({'kinematics': 'cartesian'}, section='printer')
    stepper_z_section = FakeConfig({'step_pin': 'PB6', 'dir_pin': '!PB5'}, section='stepper_z')
    return FakeConfig(values, section=section, printer=printer,
                       other_sections={'printer': printer_section,
                                       'stepper_z': stepper_z_section})


def make_z_compensate_config(printer, values, section='z_compensate'):
    return FakeConfig(values, section=section, printer=printer)


def connect(printer, mcu):
    """Simulates the two real klippy startup phases PRTouchV2/PrtouchProbe depend on:
    each MCU's own registered config callbacks (normally run once printer.cfg + all
    add_config_cmd calls are being assembled), then the klippy:connect event (normally
    fired once every module's __init__ has run, resolving cross-object lookups like
    lookup_object('toolhead'))."""
    mcu.run_config_callbacks()
    printer.send_event("klippy:connect")


def make_pres_result(pres_oid, tri_time_ticks, tri_chs, buf_cnt, samples):
    """Build the params dicts (2-per-message, result_run_pres_prtouch's real chunk size)
    needed to push `samples` (list of (tick_ticks, ch0, ch1, ch2, ch3), len multiple of 2)."""
    assert len(samples) % 2 == 0
    chunks = []
    for base in range(0, len(samples), 2):
        group = samples[base:base + 2]
        params = {
            'oid': pres_oid, 'tri_time': tri_time_ticks, 'tri_chs': tri_chs,
            'buf_cnt': buf_cnt, 'index': base,
        }
        for j, (tick, ch0, ch1, ch2, ch3) in enumerate(group):
            params['tick_%d' % j] = tick
            params['ch0_%d' % j] = ch0
            params['ch1_%d' % j] = ch1
            params['ch2_%d' % j] = ch2
            params['ch3_%d' % j] = ch3
        chunks.append(params)
    return chunks

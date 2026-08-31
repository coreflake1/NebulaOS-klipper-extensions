# NebulaOS calibration coordinator (Phase 2 calibration-framework mission).
#
# The single [nebulaos_calibration] object that owns the canonical
# NEBULAOS_* public calibration API. Slices implemented so far: standalone
# NEBULAOS_Z_OFFSET_CALIBRATE (METHOD=LOAD_CELL|MANUAL), thin delegating
# wrappers for PID/bed-mesh, and NEBULAOS_AXIS_TWIST_CALIBRATE
# (AXIS=X|Y|BOTH, METHOD=LOAD_CELL|MANUAL). NEBULAOS_AUTO_CALIBRATE, the
# guided Input Shaper/E-Steps workflows, NEBULAOS_CALIBRATION_CONTINUE/
# CANCEL, and the persistent calibration journal are NOT part of this
# slice - see the Phase 2 mission report for exactly what remains and why.
#
# ---------------------------------------------------------------------
# Axis Twist: how this stays a thin adapter around pristine upstream
# ---------------------------------------------------------------------
# klippy/extras/axis_twist_compensation.py (58bd67db..., NOT modified,
# NOT shadowed, NOT monkeypatched - see docs/NEBULAOS_PRISTINE_KLIPPER.md)
# remains fully authoritative for X/Y compensation data, interpolation,
# and simultaneous runtime correction. Its own interactive command,
# AXIS_TWIST_COMPENSATION_CALIBRATE, is used verbatim for METHOD=MANUAL.
#
# METHOD=LOAD_CELL cannot use that command directly - internally it drives
# klippy/extras/manual_probe.py's ManualProbeHelper for the human nozzle
# reference step, and this project is expressly forbidden from
# intercepting or monkeypatching that class. Instead:
#
#   1. Point generation: upstream computes calibration bed points inline,
#      inside cmd_AXIS_TWIST_COMPENSATION_CALIBRATE, with no separately
#      callable helper. _axis_twist_bed_points() below reproduces that
#      exact, tiny (linear-interpolation) formula - the minimum
#      reproduction the project's own rules explicitly allow when no
#      public helper exists - cited to the exact pinned source lines, and
#      proven byte-for-byte identical to the real function in
#      test_nebulaos_calibration.py (which imports the real pinned module
#      directly for the comparison).
#
#   2. Per-point measurement: nebulaos_probe_pair.measure_probe_nozzle_pair
#      (already used by NEBULAOS_Z_OFFSET_CALIBRATE) supplies the
#      probe/nozzle pair at each point. See _AXIS_TWIST_MATH_PARITY_NOTE
#      below for why its raw-frame result is numerically interchangeable
#      with upstream's own offset-frame measurement for this purpose.
#
#   3. Normalization + activation + config staging: NOT reimplemented at
#      all. The real, already-instantiated AxisTwistCompensation.Calibrater
#      object (reached via printer.lookup_object('axis_twist_compensation')
#      .calibrater - a plain public attribute) is handed this module's own
#      collected `results` list and told which axis just ran, then its own
#      real _finalize_calibration() method is called directly. That method
#      does the avg/normalize math, the configfile.set() staging (exact
#      upstream option names), and the live self.compensation.z_compensations/
#      zy_compensations activation - the SAME code upstream's own manual
#      workflow uses, byte-for-byte, not a reimplementation. This is the
#      "small isolated adapter ... constructing the object's own state and
#      invoking its own logic" the project's rules describe - not a patch,
#      not a shadow module, not a monkeypatch (nothing on the class or
#      module is replaced; only instance attributes already meant to hold
#      per-run state are set, exactly as the object's OWN code already
#      does internally).
#
# _AXIS_TWIST_MATH_PARITY_NOTE: upstream's own per-point result (inside
# Calibrater._manual_probe_callback_factory) is
#     z_offset = current_measured_z - mpresult.bed_z
#              = (raw_probe_trigger_z - current_probe_z_offset) - raw_nozzle_contact_z
# (probe.py's create_probe_result: bed_z = test_z - z_offset; ManualProbeHelper's
# mpresult carries no offset at all, so mpresult.bed_z IS the raw nozzle
# contact Z). This module's own per-point result, from measure_probe_nozzle_pair,
# is raw_probe_trigger_z - raw_nozzle_contact_z (no offset term). The two
# differ by exactly current_probe_z_offset - a CONSTANT across every point
# in one calibration run (the offset does not change mid-run). Calibrater
# ._finalize_calibration()'s own first step is `avg = mean(results);
# results = [avg - r for r in results]` - subtracting the mean cancels any
# constant term common to every element. So the two measurement conventions
# produce IDENTICAL normalized compensations regardless of what
# probe z_offset happens to be active at calibration time - proven
# numerically (with a deliberately nonzero z_offset) in
# test_nebulaos_calibration.py, not merely asserted here.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

from . import nebulaos_probe_pair

_MAX_ERROR_LEN = 200

# Matches upstream's own axis_twist_compensation.py DEFAULT_SAMPLE_COUNT.
_DEFAULT_AXIS_TWIST_SAMPLE_COUNT = 3


def axis_twist_bed_points(compensation, axis, sample_count):
    """Reproduces ONLY the linear-interpolation bed-point arithmetic from
    pinned upstream klippy/extras/axis_twist_compensation.py's
    Calibrater.cmd_AXIS_TWIST_COMPENSATION_CALIBRATE (58bd67db..., the
    'if axis == "X" / elif axis == "Y"' block) - upstream does not expose
    this as a separately-callable helper. `compensation` is the real,
    already-configured AxisTwistCompensation object (its calibrate_start_x/
    end_x/y and calibrate_start_y/end_y/x are the same six config values
    Calibrater's own __init__ already reads for x_start_point/x_end_point/
    y_start_point/y_end_point - this function deliberately reads them from
    the SAME object, not a second copy, so the two can never disagree).
    Returns a list of (x, y) bed-frame nozzle-target points, matching
    upstream's own bed_points list exactly (in order)."""
    if axis == 'X':
        start = (compensation.calibrate_start_x, compensation.calibrate_y)
        end = (compensation.calibrate_end_x, compensation.calibrate_y)
        if start[0] is None or end[0] is None or start[1] is None:
            raise ValueError(
                "axis_twist_compensation for X axis requires "
                "calibrate_start_x, calibrate_end_x and calibrate_y to be defined")
        axis_range = end[0] - start[0]
        interval = axis_range / (sample_count - 1)
        return [(start[0] + i * interval, start[1]) for i in range(sample_count)]
    elif axis == 'Y':
        start = (compensation.calibrate_x, compensation.calibrate_start_y)
        end = (compensation.calibrate_x, compensation.calibrate_end_y)
        if start[1] is None or end[1] is None or start[0] is None:
            raise ValueError(
                "axis_twist_compensation for Y axis requires "
                "calibrate_start_y, calibrate_end_y and calibrate_x to be defined")
        axis_range = end[1] - start[1]
        interval = axis_range / (sample_count - 1)
        return [(start[0], start[1] + i * interval) for i in range(sample_count)]
    else:
        raise ValueError("axis_twist_bed_points: axis must be 'X' or 'Y', got %r" % (axis,))


def _sanitize_error(exc):
    """Same convention as z_compensate.py's own _sanitize_calibration_error
    - str() on a Klipper error never includes a traceback/repr, so this
    just collapses whitespace and bounds length for a status field."""
    text = str(exc).strip() or exc.__class__.__name__
    text = ' '.join(text.split())
    if len(text) > _MAX_ERROR_LEN:
        text = text[:_MAX_ERROR_LEN - 3].rstrip() + '...'
    return text


class NebulaOSCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()

        self.down_min_z = config.getfloat('down_min_z', default=10., minval=1., maxval=50.)
        self.pro_cnt = config.getint('pro_cnt', default=3, minval=1)
        self.horizontal_move_z = config.getfloat(
            'horizontal_move_z', default=5., minval=1., maxval=50.)
        self.travel_speed = config.getfloat('travel_speed', default=200., above=0.)
        self.probe_lift_speed = config.getfloat('probe_lift_speed', default=20., above=0.)
        # Same conservative default and rationale as z_compensate.py's own
        # max_offset_correction_mm - a sanity ceiling, not a physical bound
        # (that is [heater_bed]/[extruder] max_temp's job for temperatures,
        # and here simply "this is a fine-tune, not a re-leveling").
        self.max_offset_correction_mm = config.getfloat(
            'max_offset_correction_mm', default=2., minval=0.1, maxval=10.)
        self.default_bed_pid_target = config.getfloat(
            'default_bed_pid_target', default=65., above=0.)
        self.default_hotend_pid_target = config.getfloat(
            'default_hotend_pid_target', default=230., above=0.)
        self.bed_mesh_profile_name = config.get(
            'bed_mesh_profile_name', default='nebulaos_calibration')
        self.axis_twist_sample_count = config.getint(
            'axis_twist_sample_count', default=_DEFAULT_AXIS_TWIST_SAMPLE_COUNT,
            minval=2)

        self.z_offset_state = 'idle'
        self.z_offset_id = 0
        self.z_offset_result = None
        self.z_offset_error = None

        # Axis Twist state - kept PER AXIS (not one shared state) so
        # AXIS=BOTH's status can distinguish X progress from Y progress,
        # and so recalibrating one axis never disturbs the other's last
        # known result in this module's own status view (the underlying
        # upstream object's own compensation arrays already have this
        # property independently - see clear_compensations(axis) - this
        # just mirrors it in the status model).
        self.axis_twist_id = 0
        self.axis_twist_method = None
        self.axis_twist_current_axis = None  # the sub-axis actively running now, or None
        self.axis_twist_sample_index = 0
        self.axis_twist_sample_total = 0
        self.axis_twist_x_state = 'idle'
        self.axis_twist_x_result = None
        self.axis_twist_x_error = None
        self.axis_twist_y_state = 'idle'
        self.axis_twist_y_result = None
        self.axis_twist_y_error = None

        self.gcode.register_command(
            'NEBULAOS_Z_OFFSET_CALIBRATE', self.cmd_z_offset_calibrate,
            desc=self.cmd_z_offset_calibrate_help)
        self.gcode.register_command(
            'NEBULAOS_PID_CALIBRATE_BED', self.cmd_pid_calibrate_bed,
            desc=self.cmd_pid_calibrate_bed_help)
        self.gcode.register_command(
            'NEBULAOS_PID_CALIBRATE_HOTEND', self.cmd_pid_calibrate_hotend,
            desc=self.cmd_pid_calibrate_hotend_help)
        self.gcode.register_command(
            'NEBULAOS_BED_MESH_CALIBRATE', self.cmd_bed_mesh_calibrate,
            desc=self.cmd_bed_mesh_calibrate_help)
        self.gcode.register_command(
            'NEBULAOS_CALIBRATION_STATUS', self.cmd_calibration_status,
            desc=self.cmd_calibration_status_help)
        self.gcode.register_command(
            'NEBULAOS_AXIS_TWIST_CALIBRATE', self.cmd_axis_twist_calibrate,
            desc=self.cmd_axis_twist_calibrate_help)

    # ------------------------------------------------------------------
    # Reference XY point
    # ------------------------------------------------------------------
    def _resolve_reference_xy(self, gcmd):
        """The bed point NEBULAOS_Z_OFFSET_CALIBRATE measures at, absent an
        explicit X=/Y= override. Deliberately REUSES [z_compensate]'s own
        already-correct, already-hardware-qualified home_x/home_y
        resolution (z_compensate.py's _resolve_z_home_xy(), which fixed a
        real 1.5mm Y-axis bug in an earlier mission by preferring
        [gcode_macro _HOMING_PARAMS]'s real homing target over a
        [bed_mesh]-center approximation) rather than re-deriving the same
        logic a second time here - one source of truth, not two. This is
        legacy-depends-on-canonical for now, the opposite of the eventual
        target shape; acceptable while [z_compensate] remains a required,
        always-included section on this project's single supported
        printer model. A future cleanup can extract this resolution into
        its own small shared helper if/when z_compensate.py is ever fully
        retired - tracked, not silently left as an accident."""
        x = gcmd.get_float('X', None)
        y = gcmd.get_float('Y', None)
        if x is not None and y is not None:
            return x, y
        zc = self.printer.lookup_object('z_compensate', None)
        if zc is None or zc.home_x is None or zc.home_y is None:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: no X=/Y= given and no "
                "[z_compensate] reference point is available - specify "
                "X=<pos> Y=<pos> explicitly")
        return zc.home_x, zc.home_y

    def get_status(self, eventtime):
        return {
            'z_offset_state': self.z_offset_state,
            'z_offset_id': self.z_offset_id,
            'z_offset_result': self.z_offset_result,
            'z_offset_error': self.z_offset_error,
            'axis_twist_id': self.axis_twist_id,
            'axis_twist_method': self.axis_twist_method,
            'axis_twist_current_axis': self.axis_twist_current_axis,
            'axis_twist_sample_index': self.axis_twist_sample_index,
            'axis_twist_sample_total': self.axis_twist_sample_total,
            'axis_twist_x_state': self.axis_twist_x_state,
            'axis_twist_x_result': self.axis_twist_x_result,
            'axis_twist_x_error': self.axis_twist_x_error,
            'axis_twist_y_state': self.axis_twist_y_state,
            'axis_twist_y_result': self.axis_twist_y_result,
            'axis_twist_y_error': self.axis_twist_y_error,
        }

    # ------------------------------------------------------------------
    # NEBULAOS_Z_OFFSET_CALIBRATE
    # ------------------------------------------------------------------
    cmd_z_offset_calibrate_help = (
        "Calibrate the BLTouch Z-offset (METHOD=LOAD_CELL|MANUAL)")

    def cmd_z_offset_calibrate(self, gcmd):
        method = gcmd.get('METHOD', 'LOAD_CELL').upper()
        if method == 'MANUAL':
            self._z_offset_calibrate_manual(gcmd)
        elif method == 'LOAD_CELL':
            self._z_offset_calibrate_load_cell(gcmd)
        else:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: unknown METHOD='%s' - "
                "expected LOAD_CELL or MANUAL" % (method,))

    def _z_offset_calibrate_manual(self, gcmd):
        """No duplicate paper-test implementation - delegates straight to
        stock PROBE_CALIBRATE, which already does exactly the "normal
        probe measurement + upstream Klipper manual nozzle reference"
        sequence this mode calls for (automatic BLTouch probe, move nozzle
        over that point, ManualProbeHelper's TESTZ/ACCEPT/ABORT, then its
        own configfile.set()). This call only starts that interactive
        session; the user continues it with TESTZ/ACCEPT/ABORT exactly as
        they would after calling PROBE_CALIBRATE directly."""
        self.gcode.respond_info(
            "NEBULAOS_Z_OFFSET_CALIBRATE: starting a manual Z-offset "
            "calibration via stock PROBE_CALIBRATE - continue with TESTZ/"
            "ACCEPT/ABORT.")
        self.gcode.run_script_from_command('PROBE_CALIBRATE')

    def _z_offset_calibrate_load_cell(self, gcmd):
        z_offset_probe = self.printer.lookup_object('nebulaos_z_offset_probe', None)
        if z_offset_probe is None:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: no [nebulaos_z_offset_probe] "
                "is configured on this printer - use METHOD=MANUAL instead, "
                "or add [nebulaos_z_offset_probe] to printer.cfg")
        if not z_offset_probe.get_status(self.reactor.monotonic())['is_calibrated']:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: the load cell has not been "
                "calibrated yet - run LOAD_CELL_CALIBRATE first, or use "
                "METHOD=MANUAL")

        x, y = self._resolve_reference_xy(gcmd)
        probe_obj = self.printer.lookup_object('probe')
        probe_x_offset, probe_y_offset, _z_offset = probe_obj.get_offsets()

        self.z_offset_id += 1
        self.z_offset_state = 'running'
        self.z_offset_result = None
        self.z_offset_error = None

        try:
            measurement = nebulaos_probe_pair.measure_probe_nozzle_pair(
                self.printer, x, y, probe_x_offset, probe_y_offset,
                self.horizontal_move_z, z_offset_probe, self.down_min_z,
                pro_cnt=self.pro_cnt, travel_speed=self.travel_speed,
                probe_lift_speed=self.probe_lift_speed)
            new_offset = measurement.probe_z_offset

            if not math.isfinite(new_offset):
                raise self.printer.command_error(
                    "NEBULAOS_Z_OFFSET_CALIBRATE: measured value %r is "
                    "not a finite number" % (new_offset,))
            if abs(new_offset) > self.max_offset_correction_mm:
                raise self.printer.command_error(
                    "NEBULAOS_Z_OFFSET_CALIBRATE: measured value %.5fmm "
                    "exceeds max_offset_correction_mm=%.5fmm - refusing "
                    "to apply an implausibly large correction"
                    % (new_offset, self.max_offset_correction_mm))

            # Effective in the CURRENT session immediately (needed before
            # Bed Mesh runs later in a future Auto-Calibrate sequence) -
            # pinned upstream Klipper exposes no public runtime command to
            # replace the registered probe's live z_offset directly (only
            # a full config reload/restart, or Z_OFFSET_APPLY_PROBE +
            # SAVE_CONFIG, which is exactly the restart this must avoid
            # mid-sequence). This is the minimum NebulaOS-side adapter for
            # that gap: confirmed directly against the pinned source
            # (klippy/extras/bltouch.py's PrinterBLTouch.__init__ does
            # `self.probe_offsets = probe.ProbeOffsetsHelper(config)`, and
            # every get_offsets() call reads that same live instance) that
            # mutating probe_obj.probe_offsets.z_offset is the correct,
            # real seam - not an upstream Klipper patch, nothing on disk
            # is touched, and it is undone automatically by the next
            # config (re)load like any other unsaved runtime state.
            probe_obj.probe_offsets.z_offset = new_offset

            configfile = self.printer.lookup_object('configfile')
            configfile.set(probe_obj.cmd_helper.name, 'z_offset',
                            "%.3f" % (new_offset,))
        except Exception as e:
            self.z_offset_state = 'error'
            self.z_offset_result = None
            self.z_offset_error = _sanitize_error(e)
            raise

        self.z_offset_state = 'complete'
        self.z_offset_result = new_offset
        self.z_offset_error = None
        self.gcode.respond_info(
            "NEBULAOS_Z_OFFSET_CALIBRATE: measured %.5f mm (probe trigger "
            "%.5f, nozzle contact %.5f at X=%.3f Y=%.3f), applied live. "
            "The SAVE_CONFIG command will make this permanent."
            % (new_offset, measurement.raw_probe_trigger_z,
               measurement.raw_nozzle_contact_z, x, y))

    # ------------------------------------------------------------------
    # NEBULAOS_AXIS_TWIST_CALIBRATE
    # ------------------------------------------------------------------
    cmd_axis_twist_calibrate_help = (
        "Calibrate Axis Twist Compensation (AXIS=X|Y|BOTH METHOD=LOAD_CELL|MANUAL)")

    def cmd_axis_twist_calibrate(self, gcmd):
        # AXIS has no default on purpose - per this project's own rules,
        # ambiguous input must never silently select a (potentially
        # unsafe/unintended) axis.
        axis = gcmd.get('AXIS', None)
        if axis is None:
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE: AXIS=X|Y|BOTH is required")
        axis = axis.upper()
        if axis not in ('X', 'Y', 'BOTH'):
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE: unknown AXIS='%s' - "
                "expected X, Y, or BOTH" % (axis,))
        method = gcmd.get('METHOD', 'LOAD_CELL').upper()
        if method not in ('LOAD_CELL', 'MANUAL'):
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE: unknown METHOD='%s' - "
                "expected LOAD_CELL or MANUAL" % (method,))
        # Resolved once, shared by both methods, matching upstream's own
        # AXIS_TWIST_COMPENSATION_CALIBRATE SAMPLE_COUNT= gcode param name
        # and semantics exactly (default: this project's own
        # axis_twist_sample_count config value, itself defaulted to
        # upstream's own DEFAULT_SAMPLE_COUNT=3).
        sample_count = gcmd.get_int('SAMPLE_COUNT', self.axis_twist_sample_count, minval=2)

        if method == 'MANUAL':
            self._axis_twist_calibrate_manual(axis, gcmd, sample_count)
            return

        axis_twist = self.printer.lookup_object('axis_twist_compensation', None)
        if axis_twist is None:
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE: [axis_twist_compensation] "
                "is not configured on this printer")
        z_offset_probe = self.printer.lookup_object('nebulaos_z_offset_probe', None)
        if z_offset_probe is None:
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE: no [nebulaos_z_offset_probe] "
                "is configured on this printer - use METHOD=MANUAL instead")
        if not z_offset_probe.get_status(self.reactor.monotonic())['is_calibrated']:
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE: the load cell has not been "
                "calibrated yet - run LOAD_CELL_CALIBRATE first, or use "
                "METHOD=MANUAL")

        self.axis_twist_id += 1
        self.axis_twist_method = 'LOAD_CELL'
        if axis in ('X', 'BOTH'):
            self._axis_twist_calibrate_load_cell_one_axis(
                'X', axis_twist, z_offset_probe, gcmd, sample_count)
        if axis in ('Y', 'BOTH'):
            self._axis_twist_calibrate_load_cell_one_axis(
                'Y', axis_twist, z_offset_probe, gcmd, sample_count)

    def _axis_twist_calibrate_manual(self, axis, gcmd, sample_count):
        """No duplicate paper-test logic, no ManualProbeHelper
        interception - delegates straight to stock
        AXIS_TWIST_COMPENSATION_CALIBRATE, exactly as the mission's own
        rules require. AXIS=BOTH is deliberately rejected rather than
        auto-chained: upstream's own command is a single-axis, fully
        interactive, multi-step wizard (TESTZ/ACCEPT/ABORT per point) with
        no seam this module may cleanly hook to detect "X finished,
        safely start Y" without intercepting upstream's own callback
        chain - which this project's rules forbid. Running X and Y
        separately is a normal, accepted two-command workflow."""
        if axis == 'BOTH':
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE AXIS=BOTH METHOD=MANUAL is "
                "not supported - run AXIS=X METHOD=MANUAL, then separately "
                "AXIS=Y METHOD=MANUAL")
        self.gcode.respond_info(
            "NEBULAOS_AXIS_TWIST_CALIBRATE: starting a manual AXIS=%s "
            "calibration via stock AXIS_TWIST_COMPENSATION_CALIBRATE - "
            "continue with TESTZ/ACCEPT/ABORT." % (axis,))
        self.gcode.run_script_from_command(
            'AXIS_TWIST_COMPENSATION_CALIBRATE AXIS=%s SAMPLE_COUNT=%d'
            % (axis, sample_count))

    def _axis_twist_calibrate_load_cell_one_axis(self, axis, axis_twist,
                                                  z_offset_probe, gcmd,
                                                  sample_count):
        probe_obj = self.printer.lookup_object('probe')
        probe_x_offset, probe_y_offset, _z_offset = probe_obj.get_offsets()

        try:
            bed_points = axis_twist_bed_points(axis_twist, axis, sample_count)
        except ValueError as e:
            self._axis_twist_set_axis_state(axis, 'error', None, _sanitize_error(e))
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE: %s" % (e,))

        self.axis_twist_current_axis = axis
        self.axis_twist_sample_index = 0
        self.axis_twist_sample_total = len(bed_points)
        self._axis_twist_set_axis_state(axis, 'running', None, None)

        # Clears ONLY this axis's own compensation array, exactly like
        # upstream's own cmd_AXIS_TWIST_COMPENSATION_CALIBRATE does before
        # collecting new samples - a real, public method on the real
        # object, not reimplemented. The other axis's compensation is left
        # completely untouched (requirement: recalibrating one axis must
        # preserve the other).
        axis_twist.clear_compensations(axis)

        try:
            results = []
            for i, (x, y) in enumerate(bed_points):
                self.axis_twist_sample_index = i + 1
                measurement = nebulaos_probe_pair.measure_probe_nozzle_pair(
                    self.printer, x, y, probe_x_offset, probe_y_offset,
                    self.horizontal_move_z, z_offset_probe, self.down_min_z,
                    pro_cnt=self.pro_cnt, travel_speed=self.travel_speed,
                    probe_lift_speed=self.probe_lift_speed)
                if not math.isfinite(measurement.probe_z_offset):
                    raise self.printer.command_error(
                        "NEBULAOS_AXIS_TWIST_CALIBRATE: AXIS=%s sample "
                        "%d/%d at X=%.3f Y=%.3f produced a non-finite "
                        "measurement" % (axis, i + 1, len(bed_points), x, y))
                results.append(measurement.probe_z_offset)

            # Hand off to the REAL upstream finalize step - see this
            # module's own header comment for the full justification. Only
            # instance attributes the object's own code already uses for
            # per-run state are set; _finalize_calibration() itself is
            # upstream's own method, called directly, not reimplemented.
            calibrater = axis_twist.calibrater
            calibrater.results = results
            calibrater.current_axis = axis
            calibrater.gcmd = gcmd
            calibrater._finalize_calibration()
        except Exception as e:
            self._axis_twist_set_axis_state(axis, 'error', None, _sanitize_error(e))
            self.axis_twist_current_axis = None
            raise

        # calibrater.results now holds the normalized (mean-centered)
        # compensations - _finalize_calibration() replaces self.results
        # with exactly that before returning.
        self._axis_twist_set_axis_state(axis, 'complete', list(calibrater.results), None)
        self.axis_twist_current_axis = None
        self.gcode.respond_info(
            "NEBULAOS_AXIS_TWIST_CALIBRATE: AXIS=%s complete, %d point(s), "
            "now active. The SAVE_CONFIG command will make this permanent."
            % (axis, len(bed_points)))

    def _axis_twist_set_axis_state(self, axis, state, result, error):
        if axis == 'X':
            self.axis_twist_x_state = state
            self.axis_twist_x_result = result
            self.axis_twist_x_error = error
        else:
            self.axis_twist_y_state = state
            self.axis_twist_y_result = result
            self.axis_twist_y_error = error

    # ------------------------------------------------------------------
    # Thin delegating wrappers (real logic stays entirely upstream)
    # ------------------------------------------------------------------
    cmd_pid_calibrate_bed_help = "Calibrate bed heater PID (stock PID_CALIBRATE)"

    def cmd_pid_calibrate_bed(self, gcmd):
        target = gcmd.get_float('TARGET', self.default_bed_pid_target)
        self.gcode.run_script_from_command(
            'PID_CALIBRATE HEATER=heater_bed TARGET=%.2f' % (target,))

    cmd_pid_calibrate_hotend_help = "Calibrate hotend PID (stock PID_CALIBRATE)"

    def cmd_pid_calibrate_hotend(self, gcmd):
        target = gcmd.get_float('TARGET', self.default_hotend_pid_target)
        self.gcode.run_script_from_command(
            'PID_CALIBRATE HEATER=extruder TARGET=%.2f' % (target,))

    cmd_bed_mesh_calibrate_help = (
        "Full reference bed mesh, saved under a named profile (stock BED_MESH_CALIBRATE)")

    def cmd_bed_mesh_calibrate(self, gcmd):
        profile = gcmd.get('PROFILE', self.bed_mesh_profile_name)
        self.gcode.run_script_from_command('BED_MESH_CALIBRATE')
        self.gcode.run_script_from_command(
            'BED_MESH_PROFILE SAVE="%s"' % (profile,))

    # ------------------------------------------------------------------
    # NEBULAOS_CALIBRATION_STATUS
    # ------------------------------------------------------------------
    cmd_calibration_status_help = "Report the current NebulaOS calibration status"

    def cmd_calibration_status(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        gcmd.respond_info(
            "NEBULAOS_CALIBRATION_STATUS: z_offset_state=%s z_offset_id=%s "
            "z_offset_result=%s z_offset_error=%s"
            % (status['z_offset_state'], status['z_offset_id'],
               status['z_offset_result'], status['z_offset_error']))
        gcmd.respond_info(
            "NEBULAOS_CALIBRATION_STATUS: axis_twist X state=%s result=%s "
            "error=%s | Y state=%s result=%s error=%s | active=%s "
            "sample=%s/%s"
            % (status['axis_twist_x_state'], status['axis_twist_x_result'],
               status['axis_twist_x_error'], status['axis_twist_y_state'],
               status['axis_twist_y_result'], status['axis_twist_y_error'],
               status['axis_twist_current_axis'],
               status['axis_twist_sample_index'],
               status['axis_twist_sample_total']))


def load_config(config):
    return NebulaOSCalibration(config)

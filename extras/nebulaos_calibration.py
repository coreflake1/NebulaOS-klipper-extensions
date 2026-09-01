# NebulaOS calibration coordinator (Phase 2 calibration-framework mission;
# contact-safety stabilization rewrite).
#
# The single [nebulaos_calibration] object that owns the canonical
# NEBULAOS_* public calibration API. Slices implemented so far: standalone
# NEBULAOS_Z_OFFSET_CALIBRATE (METHOD=LOAD_CELL|MANUAL, bounded-descent
# envelope + measurement-quality/repeatability gating), thin delegating
# wrappers for PID/bed-mesh, and NEBULAOS_AXIS_TWIST_CALIBRATE
# (AXIS=X|Y|BOTH - see "Axis Twist" section below: HARD BLOCKED pending
# remote load-cell contact hardware qualification). NEBULAOS_AUTO_CALIBRATE,
# the guided Input Shaper/E-Steps workflows, NEBULAOS_CALIBRATION_CONTINUE/
# CANCEL, and the persistent calibration journal are NOT part of this
# slice - see the Phase 2 mission report for exactly what remains and why.
#
# ---------------------------------------------------------------------
# Upstream-first cleanup (Overnight Contact-Safety Stabilization mission)
# ---------------------------------------------------------------------
# Manual Axis Twist is NO LONGER wrapped here at all. There is exactly one
# way to run it: call pristine upstream's own
# AXIS_TWIST_COMPENSATION_CALIBRATE AXIS=X|Y directly (also its own
# interactive TESTZ/ACCEPT/ABORT workflow) - this module used to offer a
# thin METHOD=MANUAL passthrough (`_axis_twist_calibrate_manual`), which
# added a second name for the exact same upstream command with no product
# value, and has been removed. klippy/extras/axis_twist_compensation.py
# (58bd67db..., NOT modified, NOT shadowed, NOT monkeypatched - see
# docs/NEBULAOS_PRISTINE_KLIPPER.md) remains fully authoritative for X/Y
# compensation data, interpolation, and runtime correction, exactly as
# before.
#
# ---------------------------------------------------------------------
# Axis Twist: automatic (LOAD_CELL) path is HARD BLOCKED
# ---------------------------------------------------------------------
# NEBULAOS_AXIS_TWIST_CALIBRATE remains NebulaOS-owned - upstream provides
# no automatic load-cell nozzle-reference frontend, and one is genuinely
# useful once the underlying contact primitive is hardware-qualified for
# REMOTE bed points (away from [nebulaos_z_offset_probe]'s own qualified
# reference point - see the real safety incident this whole mission is
# built on, _evidence/overnight-hx711-investigation-20260831-233518/
# REPORT.md). That qualification has NOT happened. Until it does, this
# command performs ZERO movement, ZERO nozzle contact, and ZERO
# compensation/config changes for any AXIS value - see
# cmd_axis_twist_calibrate() below, which returns a structured
# REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED error before looking up any
# hardware object at all. There is deliberately no unsafe public override.
#
# The corrected pure geometry-preflight math this future qualification
# will need (axis_twist_geometry_preflight() below) is implemented and
# fully tested now, but is NOT wired to any live motion path - it is dead
# code from cmd_axis_twist_calibrate's point of view until a future
# mission re-enables the LOAD_CELL path. Its own header comment explains
# the transform and the real analysis mistake (sign error) an earlier
# session made and then corrected by reading nebulaos_probe_pair.py's real
# code, not re-deriving it - the SAME subtraction-based transform is used
# in both places for exactly that reason (a second, potentially-divergent
# copy of this math is the failure mode being avoided).
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


def axis_twist_geometry_preflight(axis, bed_points, probe_x_offset,
                                   probe_y_offset, axis_minimum,
                                   axis_maximum, safety_margin_mm=0.0):
    """Pure geometry validation for an Axis Twist LOAD_CELL calibration run
    - NO printer/toolhead access, NO movement of any kind, safe to call at
    any time (including before homing). NOT currently reachable from
    cmd_axis_twist_calibrate() - see this module's own header comment on
    why the LOAD_CELL path is hard blocked - but implemented and tested
    now so a future qualification mission has a correct, ready-made check
    to wire in rather than deriving this again.

    For each nozzle-frame bed point `P` (as returned by
    axis_twist_bed_points), computes BOTH carriage targets a real
    calibration run would need to reach:
        nozzle_carriage_target = P                          (no offset)
        probe_carriage_target  = P - (probe_x_offset, probe_y_offset)
    - the SAME subtraction-based transform nebulaos_probe_pair.py's real,
    hardware-qualified measure_probe_nozzle_pair() uses (confirmed
    directly against its source, not re-derived a second, potentially-
    divergent way - see this module's header comment for why an earlier,
    ADDITION-based version of this exact check was wrong and led to a real
    live-session safety incident).

    Validates the axis this run cares about (X for axis=='X', Y for
    axis=='Y') on BOTH targets against [axis_minimum, axis_maximum],
    shrunk on both ends by safety_margin_mm. Returns the ordered list of
    (nozzle_target, probe_target) pairs on success. Raises ValueError
    naming the FIRST invalid point (and whether it was the nozzle or probe
    target) on failure - never silently clamps, skips, or adjusts an
    endpoint.
    """
    if axis not in ('X', 'Y'):
        raise ValueError(
            "axis_twist_geometry_preflight: axis must be 'X' or 'Y', got %r"
            % (axis,))
    axis_index = 0 if axis == 'X' else 1
    lo = axis_minimum + safety_margin_mm
    hi = axis_maximum - safety_margin_mm
    offset = probe_x_offset if axis == 'X' else probe_y_offset

    results = []
    for i, point in enumerate(bed_points):
        nozzle_target = point
        probe_target = (point[0] - probe_x_offset, point[1] - probe_y_offset)

        nozzle_value = nozzle_target[axis_index]
        probe_value = probe_target[axis_index]

        if not (lo <= nozzle_value <= hi):
            raise ValueError(
                "axis_twist_geometry_preflight: AXIS=%s point %d/%d - "
                "nozzle carriage target %.3f is outside [%.3f, %.3f] "
                "(safety_margin_mm=%.3f)"
                % (axis, i + 1, len(bed_points), nozzle_value, lo, hi,
                   safety_margin_mm))
        if not (lo <= probe_value <= hi):
            raise ValueError(
                "axis_twist_geometry_preflight: AXIS=%s point %d/%d - "
                "probe carriage target %.3f (offset=%.3f) is outside "
                "[%.3f, %.3f] (safety_margin_mm=%.3f)"
                % (axis, i + 1, len(bed_points), probe_value, offset, lo,
                   hi, safety_margin_mm))

        results.append((nozzle_target, probe_target))
    return results


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
        # reference_x/reference_y (Phase 2 contact-safety mission, §11):
        # the canonical physical bed point NEBULAOS_Z_OFFSET_CALIBRATE
        # LOAD_CELL measures at, absent an explicit X=/Y= override -
        # firmware config's job, not a hardcoded Python constant. Factory
        # candidate is the stock KE's own validated touch-reference region
        # (noz_pos_center: 20,25 in z_compensate.py/[z_compensate], the
        # same real, stock-precedented point the overnight investigation
        # confirmed is stock's sole validated single-strain-gauge contact
        # location - see _project/missions/
        # nebulaos-klipper-loadcell-architecture-history.md). Left unset
        # (None) here on purpose; _resolve_reference_xy() falls back to
        # [z_compensate]'s own home_x/home_y when neither this nor an
        # explicit X=/Y= is available, so an older printer.cfg without
        # these two options still works exactly as before.
        self.reference_x = config.getfloat('reference_x', default=None)
        self.reference_y = config.getfloat('reference_y', default=None)
        # Measurement-quality and repeatability acceptance bounds (§7/§9/
        # §10) - deliberately no invented production defaults. Left unset
        # (None) until real hardware qualification establishes them;
        # nebulaos_probe_pair.measure_probe_nozzle_pair() fails closed
        # with CONTACT_SAFETY_LIMIT_UNQUALIFIED rather than silently
        # applying a guessed bound. Tests may configure synthetic values.
        self.max_abs_fit_delta = config.getfloat(
            'max_abs_fit_delta', default=None, minval=0.001, maxval=10.)
        self.min_accepted_samples = config.getint(
            'min_accepted_samples', default=None, minval=1)
        self.max_repeatability_range = config.getfloat(
            'max_repeatability_range', default=None, minval=0.001, maxval=10.)
        self.max_repeatability_stddev = config.getfloat(
            'max_repeatability_stddev', default=None, minval=0.001, maxval=10.)
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
        # Structured diagnostics (§8/§22) for the most recent LOAD_CELL
        # attempt - the small, Moonraker-status-safe summary fields, NOT
        # the full per-sample ContactSample list (see get_status()'s own
        # comment on why that stays out of normal status).
        self.z_offset_physical_x = None
        self.z_offset_physical_y = None
        self.z_offset_predicted_surface_z = None
        self.z_offset_commanded_floor_z = None
        self.z_offset_raw_probe_trigger_z = None
        self.z_offset_sample_count = None
        self.z_offset_accepted_count = None
        self.z_offset_range = None
        self.z_offset_stddev = None

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
        explicit X=/Y= override.

        Phase 2 contact-safety mission (§11): the canonical reference is
        now this section's OWN configured reference_x/reference_y (the
        local sensor area, e.g. the stock-precedented 20,25 region) -
        firmware config's job, not a hardcoded Python constant and not
        borrowed from a different section's concern.

        Falls back to [z_compensate]'s own already-correct, already-
        hardware-qualified home_x/home_y resolution (z_compensate.py's
        _resolve_z_home_xy(), which fixed a real 1.5mm Y-axis bug in an
        earlier mission by preferring [gcode_macro _HOMING_PARAMS]'s real
        homing target over a [bed_mesh]-center approximation) ONLY when
        reference_x/reference_y are both unset - keeps an older
        printer.cfg without those two new options working exactly as
        before, without silently overriding an operator who has not yet
        adopted the new config keys."""
        x = gcmd.get_float('X', None)
        y = gcmd.get_float('Y', None)
        if x is not None and y is not None:
            return x, y
        if self.reference_x is not None and self.reference_y is not None:
            return self.reference_x, self.reference_y
        zc = self.printer.lookup_object('z_compensate', None)
        if zc is None or zc.home_x is None or zc.home_y is None:
            raise self.printer.command_error(
                "NEBULAOS_Z_OFFSET_CALIBRATE: no X=/Y= given, no "
                "reference_x/reference_y configured on [nebulaos_calibration], "
                "and no [z_compensate] reference point is available - "
                "specify X=<pos> Y=<pos> explicitly")
        return zc.home_x, zc.home_y

    def get_status(self, eventtime):
        return {
            'z_offset_state': self.z_offset_state,
            'z_offset_id': self.z_offset_id,
            'z_offset_result': self.z_offset_result,
            'z_offset_error': self.z_offset_error,
            'z_offset_physical_x': self.z_offset_physical_x,
            'z_offset_physical_y': self.z_offset_physical_y,
            'z_offset_predicted_surface_z': self.z_offset_predicted_surface_z,
            'z_offset_commanded_floor_z': self.z_offset_commanded_floor_z,
            'z_offset_raw_probe_trigger_z': self.z_offset_raw_probe_trigger_z,
            'z_offset_sample_count': self.z_offset_sample_count,
            'z_offset_accepted_count': self.z_offset_accepted_count,
            'z_offset_range': self.z_offset_range,
            'z_offset_stddev': self.z_offset_stddev,
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
        self.z_offset_physical_x = x
        self.z_offset_physical_y = y

        try:
            # Bounded-descent envelope, measurement-quality gates, and
            # repeatability aggregation all live inside this call now
            # (§5-§10) - measure_probe_nozzle_pair() fails closed with
            # CONTACT_SAFETY_LIMIT_UNQUALIFIED before any nozzle contact
            # motion if max_contact_descent_mm/max_abs_fit_delta/the
            # repeatability bounds are not configured (not yet hardware-
            # qualified). See nebulaos_probe_pair.py's own header.
            measurement = nebulaos_probe_pair.measure_probe_nozzle_pair(
                self.printer, x, y, probe_x_offset, probe_y_offset,
                self.horizontal_move_z, z_offset_probe, self.down_min_z,
                pro_cnt=self.pro_cnt, travel_speed=self.travel_speed,
                probe_lift_speed=self.probe_lift_speed,
                max_abs_fit_delta=self.max_abs_fit_delta,
                min_accepted_samples=self.min_accepted_samples,
                max_repeatability_range=self.max_repeatability_range,
                max_repeatability_stddev=self.max_repeatability_stddev)

            self.z_offset_predicted_surface_z = measurement.predicted_surface_z
            self.z_offset_commanded_floor_z = measurement.commanded_floor_z
            self.z_offset_raw_probe_trigger_z = measurement.raw_probe_trigger_z
            self.z_offset_sample_count = measurement.repeatability.sample_count
            self.z_offset_accepted_count = measurement.repeatability.accepted_count
            self.z_offset_range = measurement.repeatability.range
            self.z_offset_stddev = measurement.repeatability.stddev

            # §9: a rejected result (bad fit, out-of-envelope, excessive
            # fit delta, or failed repeatability) must NEVER stage a
            # Z-offset, transition to COMPLETE, or call SAVE_CONFIG - the
            # measurement's own `accepted` flag is the single
            # authoritative gate. The toolhead is already lifted clear
            # (measure_probe_nozzle_pair's own final hover()) before this
            # check runs.
            if not measurement.accepted:
                raise self.printer.command_error(
                    "NEBULAOS_Z_OFFSET_CALIBRATE: measurement rejected - "
                    "%s (samples: %d taken, %d accepted)" %
                    (measurement.rejection_reason,
                     measurement.repeatability.sample_count,
                     measurement.repeatability.accepted_count))

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
            # §22 status model: distinguish WHY a run failed where the
            # error message already tells us, rather than collapsing
            # everything to a generic 'error' - a caller polling status
            # can tell "not yet hardware-qualified" apart from "this
            # specific measurement failed quality" apart from "something
            # else broke" without parsing prose.
            text = str(e)
            if 'CONTACT_SAFETY_LIMIT_UNQUALIFIED' in text:
                self.z_offset_state = 'capability_unqualified'
            elif 'measurement rejected' in text:
                self.z_offset_state = 'measurement_quality_failure'
            else:
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
        "Automatic Axis Twist (AXIS=X|Y|BOTH) - HARD BLOCKED pending "
        "remote load-cell contact hardware qualification; use pristine "
        "upstream AXIS_TWIST_COMPENSATION_CALIBRATE for a manual run")

    _REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED_MSG = (
        "REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED: automatic Axis Twist "
        "calibration is hard-blocked - HX711 nozzle contact at a REMOTE "
        "bed point (away from [nebulaos_z_offset_probe]'s own qualified "
        "reference point) has not been hardware-qualified. Zero movement, "
        "zero nozzle contact, and zero compensation/config changes were "
        "made. For a manual calibration, call pristine upstream "
        "AXIS_TWIST_COMPENSATION_CALIBRATE AXIS=%s directly.")

    def cmd_axis_twist_calibrate(self, gcmd):
        """Phase 2 contact-safety mission (§4/§14): this command is now
        ONLY the automatic (load-cell) Axis Twist frontend - there is no
        METHOD=MANUAL passthrough any more (see this module's own header
        comment: call pristine upstream AXIS_TWIST_COMPENSATION_CALIBRATE
        directly for a manual run, it needs no wrapper). The automatic
        path itself is hard-blocked pending hardware qualification of
        remote HX711 nozzle contact: this handler performs ZERO printer
        object lookups and ZERO motion for any AXIS value - it cannot
        accidentally touch hardware even if a future edit here got the
        gating logic below wrong, because there is no hardware-touching
        code left to reach at all.
        """
        # AXIS has no default on purpose - per this project's own rules,
        # ambiguous input must never silently select a (potentially
        # unsafe/unintended) axis, even for a status/error response.
        axis = gcmd.get('AXIS', None)
        if axis is None:
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE: AXIS=X|Y|BOTH is required")
        axis = axis.upper()
        if axis not in ('X', 'Y', 'BOTH'):
            raise self.printer.command_error(
                "NEBULAOS_AXIS_TWIST_CALIBRATE: unknown AXIS='%s' - "
                "expected X, Y, or BOTH" % (axis,))

        self.axis_twist_id += 1
        self.axis_twist_method = 'LOAD_CELL'
        self.axis_twist_current_axis = None
        msg = self._REMOTE_LOAD_CELL_CONTACT_UNQUALIFIED_MSG % (
            axis if axis != 'BOTH' else 'X (and separately AXIS=Y)')
        if axis in ('X', 'BOTH'):
            self._axis_twist_set_axis_state('X', 'capability_unqualified', None, msg)
        if axis in ('Y', 'BOTH'):
            self._axis_twist_set_axis_state('Y', 'capability_unqualified', None, msg)
        raise self.printer.command_error(
            "NEBULAOS_AXIS_TWIST_CALIBRATE: %s" % (msg,))

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

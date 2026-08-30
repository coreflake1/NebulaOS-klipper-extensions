# NebulaOS automatic probe/nozzle pairing primitive (Phase 2 calibration-
# framework mission).
#
# The shared math behind both NEBULAOS_Z_OFFSET_CALIBRATE METHOD=LOAD_CELL
# and NEBULAOS_AXIS_TWIST_CALIBRATE METHOD=LOAD_CELL: at a physical bed
# point P, take a FRESH automatic BLTouch probe reading, then move the
# actual nozzle over the exact same P and take a validated HX711 contact
# reading, and derive the probe's true Z-offset from the two raw readings.
#
# Derived directly from pinned upstream Klipper (58bd67db...)'s own
# arithmetic, not invented: klippy/extras/probe.py's
# ProbeCommandHelper.probe_calibrate_finalize() computes
#     z_offset = offsets[2] - mpresult.bed_z + ppos.bed_z
# where ppos.bed_z = raw_trigger_z - current_z_offset (manual_probe.py's
# create_probe_result) and mpresult.bed_z is the RAW toolhead Z a human (or,
# here, the load cell) accepts as nozzle-bed contact. Substituting and
# simplifying, the current z_offset cancels algebraically:
#     z_offset_new = raw_probe_trigger_z - raw_nozzle_contact_z
# both measured in the SAME toolhead-Z coordinate frame. Because both
# readings are taken back-to-back in whatever gcode-offset frame happens to
# be active at the time, any currently-active SET_GCODE_OFFSET Z also
# cancels the same way - this result does not depend on it.
#
# Also independent of [axis_twist_compensation]'s state, whether or not
# that section is configured: axis twist's own _update_z_compensation_value
# (klippy/extras/axis_twist_compensation.py) only ever corrects a
# ProbeResult's .bed_z field, never .test_z (the raw trigger position this
# module reads) - confirmed directly from the pinned source, not assumed.
# ZOffsetProbe.touch_probe() bypasses the probe:update_results event
# entirely (see nebulaos_z_offset_probe.py's own header), so it was never
# affected either. There is therefore no need to clear/disable axis twist
# before measuring a Z-offset with this primitive.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections

from . import probe as probe_module

PairedMeasurement = collections.namedtuple(
    'PairedMeasurement',
    ['x', 'y', 'raw_probe_trigger_z', 'raw_nozzle_contact_z', 'probe_z_offset'])


def measure_probe_nozzle_pair(printer, x, y, probe_x_offset, probe_y_offset,
                               horizontal_move_z, z_offset_probe,
                               down_min_z, pro_cnt=1,
                               travel_speed=None, probe_lift_speed=None):
    """Take one paired (probe, nozzle) measurement at bed point (x, y).

    probe_x_offset/probe_y_offset: the registered probe's own configured
    XY offset (e.g. BLTouch's [bltouch] x_offset/y_offset) - the probe TIP
    is moved to (x - probe_x_offset, y - probe_y_offset) so that the probe
    itself ends up physically over (x, y), exactly mirroring upstream
    axis_twist_compensation.py's own _calculate_test_points().

    horizontal_move_z: a safe hover height used for every XY traverse in
    this sequence, so the toolhead never drags either the probe or the
    nozzle across the bed at print height - applied before EVERY XY move,
    not just the first.

    z_offset_probe: a nebulaos_z_offset_probe.ZOffsetProbe instance (or
    anything exposing the same touch_probe(down_min_z, pro_cnt) contract).

    travel_speed/probe_lift_speed: motion speeds for the repositioning
    moves; None lets toolhead.manual_move fall back to whatever speed was
    last used (matches upstream's own _move_helper convention in
    axis_twist_compensation.py, which only overrides speed for the initial
    hover). Callers should normally pass explicit values.

    Returns a PairedMeasurement. Raises the same exceptions
    run_single_probe()/touch_probe() would raise on their own failure
    paths (unhomed axis, sensor error, insufficient fit data, etc.) -
    deliberately does not catch or reinterpret them; the caller (a future
    NEBULAOS_Z_OFFSET_CALIBRATE/NEBULAOS_AXIS_TWIST_CALIBRATE command) owns
    turning that into structured calibration-error state.
    """
    toolhead = printer.lookup_object('toolhead')
    gcode = printer.lookup_object('gcode')
    probe_obj = printer.lookup_object('probe')

    def hover():
        toolhead.manual_move([None, None, horizontal_move_z], probe_lift_speed)

    # Probe reading: move the PROBE tip over (x, y), fresh single-point
    # automatic probe. A synthetic, parameter-free gcmd (the same pattern
    # upstream's own ProbePointsHelper._manual_probe_start uses) drives
    # run_single_probe with every probe-speed/sample-count default taken
    # from [probe]/[bltouch]'s own config, not re-guessed here.
    hover()
    toolhead.manual_move([x - probe_x_offset, y - probe_y_offset, None],
                         travel_speed)
    probe_gcmd = gcode.create_gcode_command("", "", {})
    ppos = probe_module.run_single_probe(probe_obj, probe_gcmd)
    raw_probe_trigger_z = ppos.test_z

    # Nozzle reading: move the NOZZLE (toolhead origin) over the exact same
    # (x, y) - no offset subtraction, since touch_probe() has no probe
    # offset of its own (nebulaos_z_offset_probe.py's own header: does not
    # implement Klipper's probe interface at all).
    hover()
    toolhead.manual_move([x, y, None], travel_speed)
    raw_nozzle_contact_z = z_offset_probe.touch_probe(down_min_z, pro_cnt=pro_cnt)

    # Leave the toolhead lifted clear of the bed before handing control
    # back - the caller may be about to traverse to a different XY point.
    hover()

    return PairedMeasurement(
        x=x, y=y,
        raw_probe_trigger_z=raw_probe_trigger_z,
        raw_nozzle_contact_z=raw_nozzle_contact_z,
        probe_z_offset=raw_probe_trigger_z - raw_nozzle_contact_z)

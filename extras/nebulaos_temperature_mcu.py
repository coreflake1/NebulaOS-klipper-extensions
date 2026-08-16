# MCU die-temperature support for the GD32 chips used by the Creality Ender-3 V3 KE
#
# Copyright (C) 2026  NebulaOS contributors
#
# Derived from Klipper's own klippy/extras/temperature_mcu.py:
#   Copyright (C) 2020-2024  Kevin O'Connor <kevin@koconnor.net>
# This module subclasses that file's PrinterTemperatureMCU rather than copying it, so the
# derivation is a live import and every upstream fix is inherited automatically. The GD32
# calibration constants below are the only original content.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# ---------------------------------------------------------------------------------------
# Why this module exists
# ---------------------------------------------------------------------------------------
#
# NebulaOS's shipped printer.cfg carries:
#
#     [temperature_sensor mcu_temp]
#     sensor_type: temperature_mcu
#
# Official Klipper's temperature_mcu.py dispatches on the MCU's own reported chip type and
# has no GD32 entry in its cfg_funcs table - rp2/sam/samd/same/stm32 only. Its fall-through,
# config_unknown(), does not warn and continue; it RAISES
# config_error("MCU temperature not supported on %s"). On a stock mainline Klipper with this
# printer's config, Klippy therefore refuses to start outright. That single line is the entire
# reason a GD32 story is needed at all.
#
# The KE's chips are confirmed GD32, not inferred: the Klipper dictionaries embedded in the
# firmware blobs this project carries decode to MCU=gd32f303xe (main board), gd32f303xb
# (nozzle/bed boards), and gd32e230x8, and the live device's own MCU identify string reports
# the same. See NebulaOS-firmware/docs/NEBULAOS_PRTOUCH_MCU_TIMER_FORENSICS.md.
#
# ---------------------------------------------------------------------------------------
# Why a subclass, and not a vendored copy
# ---------------------------------------------------------------------------------------
#
# The obvious approach - copy mainline's temperature_mcu.py wholesale and add three cfg_funcs
# entries - creates a permanent obligation to re-sync a 200-line file every time upstream
# touches it, and upstream HAS touched it: mainline commits 8ed426b54 and 5a2fd1009 changed
# the MCU_adc.setup_adc_callback()/setup_adc_sample() signatures this code calls. A stale
# vendored copy is exactly how that kind of change turns into a silent breakage.
#
# Subclassing avoids all of it. PrinterTemperatureMCU builds its cfg_funcs list with bound
# methods (`self.config_unknown`), and dispatches to the ('', self.config_unknown) entry only
# after every real chip prefix has failed to match. Overriding config_unknown() therefore gives
# a clean, well-defined extension point: GD32 chips reach it because mainline has no gd32
# prefix, we handle them, and every other unsupported chip still gets mainline's own error.
# None of mainline's prefixes ('rp2', 'sam3', 'sam4', 'same70', 'samd21', 'samd51', 'same5',
# 'stm32*') can shadow a 'gd32*' chip type, so the override is reached whenever it is needed
# and never when it is not.
#
# The cost of this design is one assumption on upstream: that config_unknown() remains the
# no-match fall-through. That is a far smaller and far more visible surface than a 200-line
# copy, and nebulaos_compat.py's preflight asserts the symbols this module depends on rather
# than trusting them.
#
# ---------------------------------------------------------------------------------------
# Section ordering
# ---------------------------------------------------------------------------------------
#
# heaters.setup_sensor() looks sensor_type up in a plain dict that add_sensor_factory()
# populates, so the factory must be registered before [temperature_sensor mcu_temp] is loaded.
# Klipper's only order-independent bootstrap for this is klippy/extras/temperature_sensors.cfg,
# an upstream-tracked file NebulaOS deliberately does not patch (patching it would dirty the
# Klipper checkout and make this a fork again).
#
# So registration is made robust from both directions instead:
#   * load_config() here registers the factory, so a bare [nebulaos_temperature_mcu] section
#     anywhere in printer.cfg is sufficient;
#   * nebulaos_compat.py - which the compatibility contract already requires to be the first
#     NebulaOS section - force-loads this module and verifies that every [temperature_sensor]
#     section naming a NebulaOS sensor type can actually resolve, turning an ordering mistake
#     into a precise, named preflight error instead of Klipper's bare
#     "Unknown temperature sensor 'nebulaos_temperature_mcu'".
# See docs/COMPATIBILITY.md.

import logging

from . import temperature_mcu

# The sensor_type string NebulaOS's printer.cfg selects. Deliberately not "temperature_mcu":
# that name belongs to upstream's own factory, and shadowing it would make the composed tree's
# behaviour depend on which module happened to register last.
SENSOR_TYPE = 'nebulaos_temperature_mcu'

# GD32 die-temperature calibration curves.
#
# Same shape as every entry in mainline's own table: an ADC-volts-per-degree slope, and a base
# temperature calibrated from one known (temperature, voltage) point. Both GD32 families report
# a NEGATIVE temperature coefficient - the sensor voltage falls as the die warms - which is why
# these slopes are negative where most of mainline's are positive.
#
# Carried forward verbatim from NebulaOS-klipper's own temperature_mcu.py at KLIPPER_PIN
# 9ccb2e5d, the code this printer actually ships and has run with. The values are the vendor
# datasheet figures for these parts: 25 C at 1.45 V, with -4.3 mV/C for the GD32E230 family and
# -4.1 mV/C for the GD32F303 family, against a 3.3 V reference.
#
# Keys are matched with str.startswith(), exactly as mainline matches its own, so 'gd32f303xe'
# and 'gd32f303xb' are listed separately even though they currently share a curve - listing the
# shorter 'gd32f303' instead would silently absorb any future GD32F303 variant that needs a
# different one.
GD32_CURVES = {
    'gd32e230x8': (3.3 / -.004300, 25., 1.45 / 3.3),
    'gd32f303xe': (3.3 / -.004100, 25., 1.45 / 3.3),
    'gd32f303xb': (3.3 / -.004100, 25., 1.45 / 3.3),
}


class NebulaOSTemperatureMCU(temperature_mcu.PrinterTemperatureMCU):
    """Mainline's MCU die-temperature sensor, plus the GD32 curves it has no entry for.

    Everything except chip dispatch - ADC setup, min/max range checking, the manual
    sensor_temperature1/sensor_adc1 override, status reporting - is inherited unchanged from
    upstream, so this class stays correct across upstream refactors of any of it.
    """

    def config_unknown(self):
        # Reached only after every mainline chip prefix has failed to match. Anything that is
        # not a GD32 this module knows about is upstream's problem to report, in upstream's own
        # words - do not swallow it.
        for prefix, (slope, cal_temp, cal_adc) in GD32_CURVES.items():
            if self.mcu_type.startswith(prefix):
                self.slope = slope
                self.base_temperature = self.calc_base(cal_temp, cal_adc)
                logging.info(
                    "nebulaos_temperature_mcu: applied GD32 curve for '%s'"
                    " (matched prefix '%s', slope=%.6f base=%.6f)",
                    self.mcu_type, prefix, self.slope, self.base_temperature)
                return
        raise self.printer.config_error(
            "nebulaos_temperature_mcu: no MCU die-temperature calibration for chip type '%s'."
            " NebulaOS adds curves for %s on top of the chips official Klipper supports; this"
            " MCU is neither. Either add a curve to GD32_CURVES in"
            " extras/nebulaos_temperature_mcu.py, supply sensor_temperature1/sensor_adc1 (and"
            " optionally sensor_temperature2/sensor_adc2) in the [temperature_sensor] section"
            " to calibrate it by hand, or remove the section - it is a diagnostic readout with"
            " no control-loop role."
            % (self.mcu_type, ', '.join(sorted(GD32_CURVES))))


def register_sensor_factory(printer, config):
    """Register this sensor type with [heaters], idempotently.

    Split out from load_config() so nebulaos_compat.py can guarantee registration has happened
    before any [temperature_sensor] section is loaded, without depending on a bare
    [nebulaos_temperature_mcu] section existing or on where in printer.cfg it sits.
    add_sensor_factory() is a plain dict assignment upstream, so repeat calls are harmless.
    """
    pheaters = printer.load_object(config, 'heaters')
    pheaters.add_sensor_factory(SENSOR_TYPE, NebulaOSTemperatureMCU)
    return pheaters


def load_config(config):
    register_sensor_factory(config.get_printer(), config)

# Persist input-shaper calibration results to printer.cfg (SAVE_INPUT_SHAPER)
#
# Copyright (C) 2023  ballaswag <https://github.com/ballaswag>
# Originally from ballaswag/guppyscreen, k1/k1_mods/calibrate_shaper_config.py, first
# published in commit 4a5cf94 ("initial guppy screen code commit. code at 0.0.11-beta",
# 2023-12-12).
#
# Vendored into NebulaOS unmodified except for this header, which the original file did not
# carry. See VENDORED.md. This is community-authored code, not NebulaOS's own work.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
class CalibrateShaperConfig:
    def __init__(self, config):
        self.printer = config.get_printer();
        
        shaper_type = config.get('shaper_type', 'mzv')
        self.shaper_type_x = config.get('shaper_type_x' , shaper_type)
        self.shaper_freq_x = config.getfloat('shaper_freq_x', 0., minval=0.)

        self.shaper_type_y = config.get('shaper_type_y' , shaper_type)
        self.shaper_freq_y = config.getfloat('shaper_freq_y', 0., minval=0.)

        # Register commands
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_command("SAVE_INPUT_SHAPER", self.cmd_save_input_shaper)

    def get_status(self, eventtime):
        return {}

    def cmd_save_input_shaper(self, gcmd):
        self.shaper_freq_x = gcmd.get_float('SHAPER_FREQ_X',
                                            self.shaper_freq_x, minval=0.)
        self.shaper_type_x = gcmd.get('SHAPER_TYPE_X', self.shaper_type_x)

        self.shaper_freq_y = gcmd.get_float('SHAPER_FREQ_Y',
                                            self.shaper_freq_y, minval=0.)
        self.shaper_type_y = gcmd.get('SHAPER_TYPE_Y', self.shaper_type_y)

        configfile = self.printer.lookup_object('configfile')

        configfile.set('input_shaper', 'shaper_type_x', self.shaper_type_x)
        configfile.set('input_shaper', 'shaper_freq_x',
                       '%.1f' % (self.shaper_freq_x,))

        configfile.set('input_shaper', 'shaper_type_y', self.shaper_type_y)
        configfile.set('input_shaper', 'shaper_freq_y',
                       '%.1f' % (self.shaper_freq_y,))

def load_config(config):
    return CalibrateShaperConfig(config)

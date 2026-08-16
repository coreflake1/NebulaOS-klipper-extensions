# GuppyScreen config-write helper (_GUPPY_SAVE_CONFIG / _GUPPY_DELETE_CONFIG)
#
# Copyright (C) 2024  ballaswag <https://github.com/ballaswag>
# Originally from ballaswag/guppyscreen, k1/k1_mods/guppy_config_helper.py, first published
# in commit a20edd7 ("add guppy config helper", 2024-01-17).
#
# Vendored into NebulaOS unmodified except for this header, which the original file did not
# carry. See VENDORED.md. This is community-authored code, not NebulaOS's own work.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
class GuppyConfigHelper:
    def __init__(self, config):
        self.printer = config.get_printer()
        
        # Register commands
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_command("_GUPPY_SAVE_CONFIG", self.cmd_guppy_save_config)
        gcode.register_command("_GUPPY_DELETE_CONFIG", self.cmd_guppy_delete_config)

    def cmd_guppy_save_config(self, gcmd):
        self.section = gcmd.get('SECTION', None)
        self.pairs = gcmd.get('KEY_VALUE', None)

        if self.section and self.pairs:
            configfile = self.printer.lookup_object('configfile')
            kv = self.pairs.split(',')
            d = dict(s.split(':') for s in kv)

            configfile.remove_section(self.section)

            for k, v in d.items():
                configfile.set(self.section, k, v)

    def cmd_guppy_delete_config(self, gcmd):
        self.section = gcmd.get('SECTION', None)

        if self.section:
            configfile = self.printer.lookup_object('configfile')
            configfile.remove_section(self.section)

def load_config(config):
    return GuppyConfigHelper(config)

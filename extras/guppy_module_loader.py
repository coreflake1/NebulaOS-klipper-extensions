# Runtime enable/disable of Klipper printer objects (_GUPPY_LOAD_MODULE / _GUPPY_UNLOAD_MODULE)
#
# Copyright (C) 2024  ballaswag <https://github.com/ballaswag>
# Originally from ballaswag/guppyscreen, k1/k1_mods/guppy_module_loader.py, first published
# in commit 1d7e584 ("Add tmc metrics graphs...", 2024-02-01).
#
# Vendored into NebulaOS unmodified except for this header, which the original file did not
# carry. See VENDORED.md. This is community-authored code, not NebulaOS's own work.
#
# Note on the name: this does NOT provide a general external-module loading mechanism. It
# registers two gcode commands that call printer.load_object()/printer.objects.pop() so
# GuppyScreen's TMC panel can bring [tmcstatus] up on demand. It routes through Klipper's own
# loader and is therefore subject to the same klippy/extras/ filesystem gate as everything
# else - it depends on NebulaOS's composition, it does not replace it.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os.path
import tempfile

class GuppyModuleLoader:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config

        # Register commands
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_command("_GUPPY_LOAD_MODULE", self.cmd_guppy_load_module)
        gcode.register_command("_GUPPY_UNLOAD_MODULE", self.cmd_guppy_unload_module)

    def cmd_guppy_load_module(self, gcmd):
        self.section = gcmd.get('SECTION', None)

        if self.section and self.section not in self.printer.objects:
            self.printer.load_object(self.config, self.section)

    def cmd_guppy_unload_module(self, gcmd):
        self.section = gcmd.get('SECTION', None)

        if self.section and self.section in self.printer.objects:
            self.printer.objects.pop(self.section)


def load_config(config):
    return GuppyModuleLoader(config)

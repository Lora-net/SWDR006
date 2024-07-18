# Copyright (c) 2019 Foundries.io
# Copyright (c) 2022 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

'''example_west_command.py

Example of a west extension in the example-application repository.'''

from west.commands import WestCommand  # your extension must subclass this
from west import log, util
import os

class SidPatch(WestCommand):

    def __init__(self):
        super().__init__(
            'sid-patch',               # gets stored as self.name
            'apply patch to sdk-sidewalk',  # self.help
            # self.description:
            '''\
                    sid-patch:

this command patches nordic's sdk-sidewalk to add option of
building for LR11xx transceiver''')

    def do_add_parser(self, parser_adder):
        # This is a bit of boilerplate, which allows you full control over the
        # type of argparse handling you want. The "parser_adder" argument is
        # the return value of an argparse.ArgumentParser.add_subparsers() call.
        parser = parser_adder.add_parser(self.name,
                                         help=self.help,
                                         description=self.description)

        # Add some example options using the standard argparse module API.
        #parser.add_argument('-o', '--optional', help='an optional argument')
        #parser.add_argument('required', help='a required argument')

        return parser           # gets stored as self.parser

    def do_run(self, args, unknown_args):
        # This gets called when the user runs the command, e.g.:
        #
        #   $ west my-command-name -o FOO BAR
        #   --optional is FOO
        #   required is BAR
        #log.inf('--optional is', args.optional)
        #log.inf('required is', args.required)
        log.inf("this is sid-patch " + util.west_topdir())
        sidewalk_dir = util.west_topdir() + "/sidewalk"
        os.chdir(sidewalk_dir)
        os.system("patch -p1 < ../ncs-example-application/nRF52840_sidewalk_lr11xx.diff")

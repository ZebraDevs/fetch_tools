"""
The run command runs a command remotely on a robot.

Copyright 2015 Fetch Robotics Inc.
Author: Alex Henning
"""

import sys

from ..util import ssh, add_user, add_robot

name = "run"
help_text = "Run a command on the robot"


def main(args):
    print "%s@%s$ %s" % (args.user, args.robot, args.command)
    if ssh(args.user, args.robot, args.command) != 0:
        print "ERROR: Command crashed"
        sys.exit(-1)


def add_arguments(parser):
    add_user(parser)
    add_robot(parser)
    parser.add_argument("command", action="store", default="bash",
                        help="Command and args to run remotely")

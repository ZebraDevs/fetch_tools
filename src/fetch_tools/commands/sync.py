"""
The sync command synchronizes a local workspace with the
corresponding workspace on a remote robot.

Copyright 2015 Fetch Robotics Inc.
Author: Alex Henning
"""

import os
import subprocess
import sys

from ..util import ssh, add_user, add_robot, add_workspace

name = "sync"
help_text = "Synchronize workspace"


def main(args):
    print "Syncing %s/src to %s@%s:%s/src" % (
        args.workspace, args.user, args.robot, args.relative_workspace
    )

    # Synchronize the workspaces
    proc = subprocess.Popen(
        ["rsync",
         # Following line is a trick to magically create the directory
         # if it doesn't exist.
         "--rsync-path", "mkdir -p " + args.relative_workspace + " && rsync",
         "-phErtz",
         "--delete",
         args.workspace + "/src",
         args.user + "@" + args.robot + ":" + args.relative_workspace + "/"]
    )
    proc.wait()
    if proc.returncode != 0:
        print "ERROR: Syncing failed"
        sys.exit(-1)

    # Use rosdep to install all dependencies
    if args.install_deps:
        ssh(args.user, args.robot, "rosdep update && rosdep install -r -y -a")

    # Run the build commands
    if args.build is not None:
        for build in args.build:
            build = build if build is not None else ""
            command = "source /opt/ros/" + os.getenv("ROS_DISTRO") + \
                      "/setup.bash && cd " + args.relative_workspace + \
                      " && catkin_make " + build
            if ssh(args.user, args.robot, command) != 0:
                print "ERROR: Build failed"
                sys.exit(-1)


def add_arguments(parser):
    add_user(parser)
    add_robot(parser)
    add_workspace(parser)
    parser.add_argument("--install-deps", action="store_true",
                        help="Install dependencies using rosdep")
    parser.add_argument("--build", nargs="?", action="append",
                        help="Build after syncing")

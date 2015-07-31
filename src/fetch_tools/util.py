"""
This file contains utilities for other commands to use.

Copyright 2015 Fetch Robotics Inc.
Author: Alex Henning
"""

from argcomplete.completers import ChoicesCompleter
import subprocess


def ssh(user, host, command, password=None):
    "Run the command on the remote host as the given user."
    if password is None:
        proc = subprocess.Popen(["ssh", "-t", user + "@" + host, command])
        proc.wait()
        return proc.returncode
    else:
        proc = subprocess.Popen(["sshpass", "-e",
                                 "ssh", "-t", user + "@" + host, command],
                                env={"SSHPASS": password})
        proc.wait()
        return proc.returncode


# Arguments
robots = ["freight" + str(i) for i in range(9)] + \
         ["fetch" + str(i) for i in range(7)]
users = subprocess.check_output(["awk", "-F:", "{ print $1}", "/etc/passwd"]) \
                  .split()


def add_user(parser):
    arg = parser.add_argument("--user", action="store",
                              help="User account to use on robot")
    arg.completer = ChoicesCompleter(users)


def add_robot(parser):
    arg = parser.add_argument("--robot", action="store", help="Robot to use")
    arg.completer = ChoicesCompleter(robots)


def add_workspace(parser):
    parser.add_argument("--workspace", action="store",
                        help="Catkin worspace to use")

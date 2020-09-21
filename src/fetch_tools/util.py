"""
This file contains utilities for other commands to use.

Copyright 2015 Fetch Robotics Inc.
Author: Alex Henning
"""

from argcomplete.completers import ChoicesCompleter
import subprocess
import sys


def ssh(user, host, command, password=None, fname=None):
    "Run the command on the remote host as the given user."

    userhost = user + "@" + host
    ssh_command = ["ssh", "-t", userhost, command]

    e_vars = None
    if password:
        ssh_command = ["sshpass", "-e"] + ssh_command
        e_vars = {"SSHPASS": password}
        # Notify user if they're missing sshpass
        if subprocess.call(["which", "sshpass"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL):
            sys.exit("\nERROR: You need `sshpass` in order to supply passwords.\n"
                     "Please run the following to install it:\n\tapt install sshpass")

    pipe = open(fname + ".txt", 'w') if fname else None

    proc = subprocess.Popen(ssh_command, env=e_vars, stdout=pipe, stderr=pipe)
    proc.wait()
    if fname:
        pipe.close()
    return proc.returncode


def run(command):
    proc = subprocess.Popen(["bash", "-c", command])
    proc.wait()
    return proc.returncode


# Arguments
def RobotCompleter(prefix, **kwargs):
    options = []
    if "fetch".startswith(prefix) and prefix != "fetch":
        options.extend("fetch" + str(i) for i in range(10))
    if "freight".startswith(prefix) and prefix != "freight":
        options.extend("freight" + str(i) for i in range(10))
    if options:
        return options
    return (prefix + str(i) for i in range(10))

users = subprocess.check_output(["awk", "-F:", "{ print $1}", "/etc/passwd"], encoding='utf-8').split()


def add_user(parser):
    arg = parser.add_argument("--user", action="store",
                              help="User account to use on robot")
    arg.completer = ChoicesCompleter(users)


def add_robot(parser):
    arg = parser.add_argument("--robot", action="store", help="Robot to use")
    arg.completer = RobotCompleter


def add_workspace(parser):
    parser.add_argument("--workspace", action="store",
                        help="Catkin workspace to use")
    parser.add_argument("--remote-workspace", action="store",
                        help="Catkin workspace to use on the robot")

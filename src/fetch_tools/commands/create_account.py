"""
The create account command creates a new user account on the specified robot.

Copyright 2015 Fetch Robotics Inc.
Author: Alex Henning
"""

import os
import subprocess
import sys
from copy import deepcopy
from getpass import getpass
from pipes import quote

from ..util import ssh, add_user, add_robot, add_workspace

name = "create-account"
help_text = "Create an account on a robot"

create_user_script = """
echo %(fetch_password)s | sudo -S adduser %(user)s --gecos '%(fullname)s,,,' --disabled-password &&
echo '%(user)s:%(password)s' | sudo chpasswd &&
sudo usermod -G adm,cdrom,sudo,dip,plugdev,lpadmin,sambashare %(user)s
"""  # noqa

setup_user_script = """
export FETCH_WORKSPACE=%(relative_workspace)s
export ROS_DISTRO=%(ros_distro)s
echo source /opt/ros/%(ros_distro)s/setup.bash >> ~/.bashrc.d/40-ros-setup.sh
echo source %(relative_workspace)s/devel/setup.bash >> ~/.bashrc.d/40-ros-setup.sh
echo '%(password)s' | sudo -S echo granting root priveleges for installation
bash ~/initialize.sh
"""  # noqa


def main(args):
    fullname = args.fullname if args.fullname is not None else args.user
    print "Creating account %s@%s for %s" % (args.user, args.robot, fullname)

    # Get fetch password for setup
    fetch_password = args.fetch_password[-1]
    if fetch_password is None:
        fetch_password = getpass(prompt="Fetch password: ")

    # Prompt for users password
    password = args.password
    if password is None:
        check_password = "Not the same"
        while password != check_password:
            if password is not None:
                print "Passwords don't match, please try again."
            password = getpass(prompt="Password: ")
            check_password = getpass(prompt="Password (confirm): ")

    # Common values to substitute into scripts
    env = {
        "fetch_password": fetch_password,
        "password": password,
        "user": args.user,
        "fullname": fullname,
        "relative_workspace": args.relative_workspace,
        "ros_distro": os.getenv("ROS_DISTRO"),
    }

    # Automatically add robot to list of known_hosts, pinning the
    # current security certificate.
    # TODO(security): Force people to verify keys?
    proc = subprocess.Popen(["sshpass", "-e",
                             "ssh", "-o", "StrictHostKeyChecking=no",
                             "fetch@" + args.robot, "exit"],
                            env={"SSHPASS": fetch_password})
    proc.wait()
    if proc.returncode != 0:
        print "ERROR: Could not add robot to known hosts"
        sys.exit(-1)

    # Create the user
    if ssh("fetch", args.robot, create_user_script % env, fetch_password) != 0:
        print "ERROR: Creating user failed"
        sys.exit(-1)

    # Copy SSH ID
    # TODO(enhancement): Allow copying of custom ids and of no id
    dd = deepcopy(os.environ)
    dd["SSHPASS"] = password
    proc = subprocess.Popen(["sshpass", "-e",
                             "ssh-copy-id", args.user + "@" + args.robot],
                            env=dd)
    proc.wait()
    if proc.returncode != 0:
        print "WARNING: Copying ID failed, continuing anyways"
        print "To manually copy an ID, first create one " \
            "https://help.github.com/articles/generating-ssh-keys/ " \
            "and then run `ssh-copy-id " + args.user + "@" + args.robot + "`"

    # Copy over skeleton setup
    skeleton = args.skeleton
    if skeleton is None:
        user_skeleton = os.getenv("HOME") + "/.fetch/robot_skeleton"
        if os.path.isdir(user_skeleton):
            skeleton = user_skeleton
        else:
            package_dir = subprocess.check_output(["rospack",
                                                   "find",
                                                   "fetch_tools"]).strip()
            skeleton = os.path.join(os.path.dirname(__file__),
                                    package_dir + "/resources/robot_skeleton")
    proc = subprocess.Popen(["scp", "-r", ".", args.user + "@" +
                             args.robot + ":~/"],
                            cwd=skeleton)
    proc.wait()
    if proc.returncode != 0:
        print "ERROR: Could not copy skeleton directory"
        sys.exit(-1)

    # Run setup script
    if ssh(args.user, args.robot, setup_user_script % env) != 0:
        print "ERRROR: Setting up user failed"
        sys.exit(-1)


def add_arguments(parser):
    add_user(parser)
    add_robot(parser)
    add_workspace(parser)
    parser.add_argument(
        "--fetch-password", nargs="?", action="append", default=["robotics"],
        help="Password for the fetch account (or blank to prompt)"
    )
    parser.add_argument("--password", nargs="?", action="store",
                        help="Password for the new account")
    parser.add_argument("--fullname", action="store", help="Users full name")
    parser.add_argument("--skeleton", action="store",
                        help="Skeleton directory to setup account")

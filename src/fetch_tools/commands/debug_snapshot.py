"""
The snapshot command creates a zip file of relevant robot debug info.

Copyright 2016-2020 Fetch Robotics Inc.
Authors: Aaron Blasdel, Eric Relson
"""

import shutil
import os
import tempfile
import subprocess
import datetime

from ..util import ssh, add_robot

name = "debug-snapshot"
help_text = "Take a debug snapshot of a running robot"

snapshot_version = "0.0.2"

fileroot = "debug_snapshot"
fileroot = '_' + fileroot + '_'

bagname = 'robot.bag'

# all ros command must source setup.bash
rosbash = 'source /opt/ros/noetic/setup.bash;'
devnull = open(os.devnull, 'w+')

topics = ["/robot_state",
          "/diagnostics",
          "/diagnostics_agg",
          "/battery_state"]

# All commands requiring sudo must be added in main
commands = {"dpkg_fetch": "COLUMNS=200 dpkg -l ros-noetic-fetch-*",
            "dpkg_ros": "COLUMNS=200 dpkg -l ros-noetic-*",
            "dpkg_all": "COLUMNS=200 dpkg -l",
            "lsusb": "lsusb -v",
            "lspci": "lspci -vv",
            "roswtf": rosbash + "roswtf",
            "rosnode_list": rosbash + "rosnode list",
            "rostopic_list": rosbash + "rostopic list",
            "rossrv_list": rosbash + "rosservice list",
            "rosparam_list": rosbash + "rosparam list",
            "rosparam_dump": rosbash + "rosparam get /",
            "dmesg": "dmesg",
            "syslog": "cat /var/log/syslog",
            "date": "date",
            "meminfo": "cat /proc/meminfo",
            "ip_route": "ip route",
            "ifconfig": "ifconfig -a",
            "iwconfig": "iwconfig",
            "network_interfaces": "cat /etc/network/interfaces",
            "netplan_full": "cat /etc/netplan/01-network-manager-all.yaml /etc/netplan/99-fetch-ethernet.yaml",
            "hosts": "cat /etc/hosts",
            "env": "env",
            "roscore.service": "cat /lib/systemd/system/roscore.service",
            "roscore.service_status": "service roscore status",
            "robot.service": "cat /lib/systemd/system/robot.service",
            "robot.service_status": "service robot status",
            "robot.journalctl": "journalctl -u robot",
            "ps3joy.service": "cat /lib/systemd/system/ps3joy.service",
            "ps3joy.service_status": "service ps3joy status",
            "ps4joy.service": "cat /lib/systemd/system/ps4joy.service",
            "ps4joy.service_status": "service ps4joy status",
            "read_board_charger": rosbash + "rosrun fetch_drivers read_board 0x3f",
            "read_board_mainboard": rosbash + "rosrun fetch_drivers read_board 0x00",
            "read_board_wheel_left": rosbash + "rosrun fetch_drivers read_board 0x11",
            "read_board_wheel_right": rosbash + "rosrun fetch_drivers read_board 0x12",
            "read_board_gripper": rosbash + "rosrun fetch_drivers read_board 0x80",
            "battery_state": rosbash + "rostopic echo -n 1 /battery_state",
            "robot_state": rosbash + "rostopic echo -n 1 /robot_state",
            "top": "top -n 1 -b",
           }


def main(args):
    # all commands requiring sudo must prepend this
    sudostr = "echo %s | sudo -S " % args.fetch_password[-1]

    commands.update({"robot_log":sudostr + "cat /var/log/ros/robot.log"})

    print('Running debug snapshot tool.')
    dirpath = tempfile.mkdtemp()
    print('... storing temporary files in: {0}'.format(dirpath))

    # Start bag recording
    bag_process = ['rosbag', 'record', '-q', '--duration=10', '-j',
                   '-O', bagname] + topics
    bag = subprocess.Popen(bag_process, cwd=dirpath, stdout=devnull)

    # Record version file
    with open(dirpath + '/version.txt', 'w') as version_file:
        version_file.write(snapshot_version)

    # Execute remote commands on robot
    for key, value in commands.items():
        print("Creating '%s.txt'." % key)
        ssh('fetch', args.robot, value, fname=dirpath + '/' + key,
            password=args.fetch_password[-1])

    print('Waiting for data gathering to complete')

    # Wait for the rosbag record parent process to exit cleanly
    bag.wait()

    # Zip directory
    timestr = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    filename = args.robot + fileroot + timestr + '.zip'

    print('Data gathering complete. Creating %s' % filename)
    proc = subprocess.Popen(["zip", "-r", filename, dirpath],
                            stdout=devnull,
                            stderr=devnull)
    proc.wait()

    # remove temp directory
    shutil.rmtree(dirpath)

    if proc.returncode == 0:
        print("\nCreated %s which you can send to Fetch support" % filename)
    else:
        print("\nERROR: failed to zip directory: %s" % dirpath)

def add_arguments(parser):
    parser.add_argument(
        "--fetch-password", nargs="?", action="append", default=["robotics"],
        help="Password for the fetch account (or blank to prompt)"
    )
    add_robot(parser)

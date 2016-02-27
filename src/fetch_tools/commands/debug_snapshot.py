"""
The snapshot command creates a zip file of relevant robot debug info.

Copyright 2016 Fetch Robotics Inc.
Author: Aaron Blasdel
"""

import shutil
import os
import tempfile
import subprocess
import datetime
import time

from ..util import ssh, add_robot

name = "debug-snapshot"
help_text = "Take a debug snapshot of a running robot"

snapshot_version = "0.0.1"

fileroot = "debug_snapshot"
fileroot = '_' + fileroot + '_'

bagname = 'robot.bag'

# all ros command must source setup.bash
rosbash = 'source /opt/ros/indigo/setup.bash;'
devnull = open(os.devnull, 'w+')

topics = ["/robot_state",
          "/diagnostics",
          "/diagnostics_agg",
          "/battery_state"]

# All commands requiring sudo must be added in main
commands = {"dpkg_fetch":"dpkg -l ros-indigo-fetch-*",
            "dpkg_all":"dpkg -l",
            "lsusb":"lsusb",
            "lspci":"lspci",
            "roswtf":rosbash + "roswtf",
            "rosnode_list":rosbash + "rosnode list",
            "rostopic_list":rosbash + "rostopic list",
            "rossrv_list":rosbash + "rosservice list",
            "rosparam_list":rosbash + "rosparam list",
            "rosparam_dump":rosbash + "rosparam get /",
            "dmesg":"dmesg",
            "date":"date",
            "meminfo":"cat /proc/meminfo",
            "ip_route":"ip route",
            "ifconfig":"ifconfig -a",
            "iwconfig":"iwconfig",
            "network_interfaces":"cat /etc/network/interfaces",
            "hosts":"cat /etc/hosts",
            "env":"env",
           }


def main(args):
    # all commands requiring sudo must prepend this
    sudostr = "echo %s | sudo -S " % args.fetch_password[-1]

    commands.update({"robot_log":sudostr + "cat /var/log/upstart/robot.log"})

    print 'Running debug snapshot tool (this should take around 10 seconds).'
    dirpath = tempfile.mkdtemp()

    # Start bag recording
    bag_start = time.time()
    bag_process = ['rosbag', 'record', '-q', '-j', '-O', bagname] + topics
    bag = subprocess.Popen(bag_process, cwd=dirpath, stdout=devnull)

    # Record version file
    with open(dirpath + '/version.txt', 'w') as version_file:
        version_file.write(snapshot_version)

    # Execute remote commands on robot
    for key, value in commands.iteritems():
        print "Creating '%s.txt'." % key
        ssh('fetch', args.robot, value, fname=dirpath + '/' + key,
            password=args.fetch_password[-1])

    print 'Waiting for data gathering to complete'
    while time.time() < bag_start + 10:
         # Wait for a 10 second bag file
        time.sleep(0.5)

    # Kill all rosbag record children
    # Note: Killing parent rosbag results in a failure to clean up
    node_proc = subprocess.Popen("rosnode list", shell=True,
                                 stdout=subprocess.PIPE)
    output = node_proc.stdout.read()
    node_proc.wait()
    for nodename in output.split("\n"):
        if (nodename.startswith('/record')):
            subprocess.Popen(["rosnode", "kill", nodename],
                             stdout=devnull,
                             stderr=devnull)

    # Wait for the rosbag record parent process to exit cleanly
    bag.wait()

    # Zip directory
    timestr = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    filename = args.robot + fileroot + timestr + '.zip'

    print 'Data gathering complete creating %s' % filename
    proc = subprocess.Popen(["zip", "-r", filename, dirpath],
                            stdout=devnull,
                            stderr=devnull)
    proc.wait()

    # remove temp directory
    shutil.rmtree(dirpath)

    if proc.returncode == 0:
        print "Created %s" % filename
    else:
        print "ERROR: failed to zip directory: %s" % dirpath

def add_arguments(parser):
    parser.add_argument(
        "--fetch-password", nargs="?", action="append", default=["robotics"],
        help="Password for the fetch account (or blank to prompt)"
    )
    add_robot(parser)

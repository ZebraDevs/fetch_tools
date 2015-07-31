"""
The lint command lints a package.

Copyright 2015 Fetch Robotics Inc.
Author: Alex Henning
"""

import fnmatch
import os
import subprocess
import sys

name = "lint"
help_text = "Lint a package in your workspace"


def main(args):
    # TODO(enhancement): Allow level of detail to be controlled
    # TODO(enhancement): Allow catkin_lint to be turned off
    # TODO(enhancement): Allow C++ to be turned off
    # TODO(enhancement): Allow python to be turned off

    if os.path.isfile(args.package):
        print "Linting file %s" % (args.package)
        print "-------------" + ("-" * len(args.package)) + "\n"
        for ending in cpp_endings:
            if args.package.endswith(ending):
                sys.exit(cpplint([args.package]))
        if args.package.endswith(".py"):
            sys.exit(pep8([args.package]))
        print "ERROR: can't lint file"
        sys.exit(1)
    elif os.path.isdir(args.package):
        print "Linting directory %s" % (args.package)
        print "------------------" + ("-" * len(args.package)) + "\n"
        sys.exit(lint_directory(args.package))
    else:
        print "Linting package %s" % (args.package)
        print "----------------" + ("-" * len(args.package)) + "\n"
        sys.exit(lint_package(args.package))


def add_arguments(parser):
    parser.add_argument("package", action="store",
                        metavar="FILE|DIRECTORY|PACKAGE",
                        help="File, directory or ROS package to lint")


def merge(c1, c2):
    return c2 if c2 != 0 else c1


def lint_package(package):
    proc = subprocess.Popen(["catkin_lint", "--pkg", package,
                             "-W2", "--explain"])
    proc.wait()
    if proc.returncode != 0:
        print "WARNING: Catkin lint failed"
    returncode = proc.returncode
    print

    package_directory = subprocess.check_output(["rospack", "find", package]) \
                                  .strip()
    return merge(lint_directory(package_directory), returncode)


def lint_directory(directory):
    cpp_files = []
    for root, _, filenames in os.walk(directory):
        for ending in cpp_endings:
            for filename in fnmatch.filter(filenames, ending):
                cpp_files.append(os.path.join(root, filename))
    returncode = cpplint(cpp_files)
    print

    py_files = []
    for root, _, filenames in os.walk(directory):
        for filename in fnmatch.filter(filenames, "*.py"):
            py_files.append(os.path.join(root, filename))
    returncode = merge(pep8(py_files), returncode)
    return returncode


cpp_endings = ["*.c", "*.cpp", "*.cc", "*.h", "*.hpp", "*.hh"]


def cpplint(files):
    if files:
        proc = subprocess.Popen([
            "/opt/ros/indigo/lib/roslint/cpplint",
            "--counting=detailed",
            "--filter=+,-runtime/references,-runtime/threadsafe_fn",
        ] + files)
        proc.wait()
        if proc.returncode != 0:
            print "WARNING: C++ lint failed"
        return proc.returncode
    return 0


def pep8(files):
    acceptable = [
        "R0902",  # Allow for many class attributes.
        "C0111",  # Docstrings in ## style (required by Doxygen) are not.
                  # recognized, so ignore warning that they are missing.
        "C0103",  # Allows 'constant' names to be lower case. This is normal
                  # in the if __name__ == '__main__' block.
        "C0325",  # Don't complain about print() vs print.
        "W0142",  # ** arguments are ok. Just a little "magic".
        "R0913",  # Allow for more than 4 arguments to _init__.
        "R0903",  # Allow classes to have one or two public methods.
    ]
    if files:
        proc = subprocess.Popen([
            "/opt/ros/indigo/lib/roslint/pep8",
            "--ignore=" + ",".join(acceptable),
            "--statistics",
            "--count"
        ] + files)
        proc.wait()
        if proc.returncode != 0:
            print "WARNING: Python lint failed"
        return proc.returncode
    return 0

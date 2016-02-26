"""
The workspace status command produces a github markdown compatible
table of the current worspace.

Copyright 2015 Fetch Robotics Inc.
Author: Alex Henning
"""

import os
import subprocess

from ..util import add_workspace

name = "workspace-status"
help_text = "Print the status of the current workspace"


def main(args):
    print "Status of %s: " % (args.workspace+"/src")
    data = []
    for name in [i for i in os.listdir(args.workspace+"/src")
                 if os.path.isdir(args.workspace+"/src/"+i)]:
        dir = args.workspace+"/src/"+name
        if os.path.isdir(dir+"/.git"):
            branch = subprocess.check_output("cd %s && git rev-parse --abbrev-ref HEAD" % dir, shell=True).strip()
            sha = subprocess.check_output("cd %s && git describe --always --dirty" % dir, shell=True).strip()
            data.append((name, branch, sha))
        else:
            data.append((name, "None", "untracked"))

    data.sort()
    name_len = max([len(name) for name, _, _ in data])
    branch_len = max([len(branch) for _, branch, _ in data])
    sha_len = max([len(sha) for _, _, sha in data])

    print "%s%s | %s%s | %s%s" % ("Name", " "*(name_len-len("name")),
                                  "Branch", " "*(branch_len-len("branch")),
                                  "SHA1", " "*(sha_len-len("sha1")))
    print "%s|%s|%s" % ("-"*(1+name_len), "-"*(2+branch_len), "-"*(1+sha_len))
    for name, branch, sha in data:
        print "%s%s | %s%s | %s%s" % (name, " "*(name_len-len(name)),
                                      branch, " "*(branch_len-len(branch)),
                                      sha, " "*(sha_len-len(sha)),)

def add_arguments(parser):
    add_workspace(parser)

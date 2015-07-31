Fetch Tools
===========

This package contains the `fetch` utility to make development
easier. For full details of command usage and arguments, run `fetch
-h` and `fetch COMMAND -h`. Below summarizes installation and common
usage.

Intended Workflow
-----------------

This tool was written to help make it easier to develop code and
switch between robots with ease. It works by inferring what robot
you're working on from `$ROS_MASTER_URI` and automating common tasks
such as account creation, code syncing, and running commands. It
assumes that you always edit code in your ROS Workspace and then
synchronize the robot with your workspace afterwards, so that you can
run it. This keeps all code that you are developing on your computer
in case someone takes it.

### Example Workflow

```
# Start working on freight 0 for the first time
ufr 0
fetch create-account --fullname "Not A. Robot"
fetch sync --install-deps --build
fetch run "roslaunch my_awesome_package do_stuff.launch"
# Edit code to fix a bug
fetch sync --build "--pkg my_awesome_package"
fetch run "roslaunch my_awesome_package do_stuff.launch"
# Switch over to freight 8, since someone kidnapped freight 0
ufr 8
fetch create-account --fullname "Not A. Robot"
fetch sync --install-deps --build
fetch run "roslaunch my_awesome_package do_stuff.launch"
```

Installation
------------

To install run, checkout the fetchrobotics/sandbox repo and run:

```
sudo apt-get install ros-indigo-fetch-tools
```

Afterwards, restart your terminal or run `source $(rospack find
fetch_tools)/setup.bash`.

Common Arguments
-------------

### Robot

Most operations interact with a robot, in this case they take an
optional `--robot` argument.  If the robot parameter is passed in
using `--robot`, that is the value used. If not, the parameter
defaults to the robot pointed at by `$ROS_MASTER_URI`. To set the
default value, add the line `export FETCH_ROBOT=myrobot` to your
.bashrc file.

### Workspace

Some operations interact with a workspace, in this case take an
optional `--workspace` argument.  If the a parameter is passed in that
value is used. If not, the parameter defaults to `~/$ROS_DISTRO`. To
set the default value, add the line `export
FETCH_WORKSPACE=/path/to/my/workspace` to your .bashrc file.

### User

Most operations interact with a robot, in this case they often effect
a particular user on that robot, so they take an optional `--user`
argument.  If the robot parameter is passed in using `--user`, that is
the value used. If not, the parameter defaults to `$USER`. To set the
default value, add the line `export FETCH_USER=myuser` to your .bashrc
file.

`fetch` Commands
----------------

### fetch create-account

To create an account:

```
fetch create-account
```

To create an account with another username:

```
fetch create-account --robot freight0 --user test
```

Once it is done you can run `ssh $USER@freight0` without entering your
password. (If both users are the same you can just run `ssh freight0`)

#### Customizing Account Creation

If you are creating accounts on lots of robots, you can customize the
install process. First, create a copy of the skeleton directory:

```
cp -r $(rospack find fetch_tools)/resources/robot_skeleton ~/.fetch/robot_skeleton
```

Next, customize all files to your desired value. `initialize.sh` is a
special script that gets run during the account creation process. If
you want to install software or run other commands every time you
create an account, customize `initialize.sh`.

### fetch sync

**WARNING:** Synchronize in this context means make the remote
workspace like the current workspace. This means new files will be
added and *old files will be deleted.* It is strongly recommended that
you never change files in the workspace on the robot directly.

To synchronize your `$workspace/src` directory with the matching
directory on another robot:

```
fetch sync
```

To synchronize another workspace with a robot:

```
fetch_sync --workspace ~/test_ws
```

To synchronize and then run `catkin_make` to build:

```
fetch sync --build
```

To synchronize and install all dependencies with rosdep:

```
fetch sync --install-deps
```

To synchronize and then run `catkin_make --pkg follow_pick
-DCMAKE_BUILD_TYPE=Debug` to build:

```
fetch sync --build "--pkg follow_pick -DCMAKE_BUILD_TYPE=Debug"
```

### fetch run

To run a command on the robot:

```
fetch run COMMAND
```

To run a command on the robot with arguments:

```
fetch run "COMMAND --flag1 --flag2 arg1"
```

### fetch lint

Fetch lint runs pep8, roslint, and catkin_lint to make sure a package
is compliant with all style guides. To lint a package:

```
fetch lint PACKAGE
```

To lint a folder:

```
fetch lint FOLDER
```

To lint a file:

```
fetch lint FILE
```

Useful Aliases
--------------

In addition to the `fetch` command, setup.bash loads some useful
aliases for working with multiple Fetch and Freight robots.

### Working with `$ROS_MASTER_URI`

There are three aliases for working with `$ROS_MASTER_URI`:

```
uf 5   # Set ROS_MASTER_URI="http://fetch5:11311"
ufr 0  # Set ROS_MASTER_URI="http://freight0:11311"
ul     # Set ROS_MASTER_URI="http://localhost:11311"
```

- `uf` is short for use fetch
- `ufr` is short for use freight
- `ul` is short for use localhost

### SSH

There are two aliases for sshing into robots

```
sf 5   # SSH into fetch 5
sfr 0  # SSH into freight0
```


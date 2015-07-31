# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
[ -z "$PS1" ] && return

# Load all files from .shell/bashrc.d directory
if [ -d $HOME/.bashrc.d ]; then
  for file in $HOME/.bashrc.d/*.sh; do
    source $file
  done
fi

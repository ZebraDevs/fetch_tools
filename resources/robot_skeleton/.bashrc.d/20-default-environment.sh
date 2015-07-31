alias -- -='cd -'
alias ..='cd ..'
alias ...='cd ../..'
alias ....='cd ../../..'
alias .....='cd ../../../..'

# Add local bin
if [ -d ~/.local/bin ]; then
    PATH=$PATH:~/.local/bin
fi

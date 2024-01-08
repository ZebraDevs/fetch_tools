eval "$(register-python-argcomplete3 fetch)"

uf () {
    if [[ $@ ]]; then
        export ROS_MASTER_URI="http://fetch$@:11311"
    fi;
    echo $ROS_MASTER_URI;
}

ufr () {
    if [[ $@ ]]; then
        export ROS_MASTER_URI="http://freight$@:11311"
    fi;
    echo $ROS_MASTER_URI;
}

ul () {
    if [[ $@ ]]; then
        export ROS_MASTER_URI="http://localhost:$@"
    else
        export ROS_MASTER_URI="http://localhost:11311"
    fi;
    echo $ROS_MASTER_URI;
}

sf () {
    if [[ $@ && $FETCH_USER ]]; then
        ssh $FETCH_USER@fetch$@
    elif [[ $@ ]]; then
        ssh fetch$@
    fi;
}

sfr () {
    if [[ $@ && $FETCH_USER ]]; then
        ssh $FETCH_USER@freight$@
    elif [[ $@ ]]; then
        ssh freight$@
    fi;
}

kf () {
    if [[ $@ && $FETCH_USER ]]; then
        ssh-copy-id ${@: 1:$(($# - 1))} $FETCH_USER@fetch${@: -1}
    elif [[ $@ ]]; then
        ssh-copy-id ${@: 1:$(($# - 1))} fetch${@: -1}
    fi;
}

kfr () {
    if [[ $@ && $FETCH_USER ]]; then
        ssh-copy-id ${@: 1:$(($# - 1))} $FETCH_USER@freight${@: -1}
    elif [[ $@ ]]; then
        ssh-copy-id ${@: 1:$(($# - 1))} freight${@: -1}
    fi;
}

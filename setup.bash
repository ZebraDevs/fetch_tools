eval "$(register-python-argcomplete fetch)"

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

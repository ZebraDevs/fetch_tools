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
    if [[ $@ ]]; then
        ssh fetch$@
    fi;
}

sfr () {
    if [[ $@ ]]; then
        ssh freight$@
    fi;
}

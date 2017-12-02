#!/usr/bin/env bash

source ~/.xcenv

touch .xc_id

CATKIN_WS_SRC=${CATKIN_WS}/src
CONTAINER_ID=$(cat ~/.xc_id)

# Verify that program is run by root.
function verify_sudo {
    if [ $(whoami) != "root" ]
    then
	echo "Must be run as root!"
	exit 1
    fi
}

# Verify that an associated container exists.
function verify_associated_container {
    if [ "$CONTAINER_ID" == "" ]
    then
       echo "No associated container!"
       exit 1
    fi
}

# Verify that the associated container is running.
function verify_running_container {
    verify_associated_container
    if [ $(docker inspect -f {{.State.Running}} $CONTAINER_ID) == "false" ]
    then
       echo "Associated container not running!"
       exit 1
    fi
}

# Verify no container is running
function verify_no_running_container {
    if [ $(docker inspect -f {{.State.Running}} $CONTAINER_ID) == "true" ]
    then
       echo "Associated container already running!"
       exit 1
    fi
}

# Print usage information
function echo_help {
    echo "Usage:"
    echo "    xc start -- Start compilation container"
    echo "    xc make  -- Build ARM binaries"
    echo "    xc push  -- Upload ARM binaries to robot" 
    echo "    xc stop  -- Stop compilation container"
}

#verify_sudo

if [ $# -eq 0 ]
then
    echo_help
    exit 1
fi

if [ $1 == "start" ] # Start container.
then
    verify_no_running_container
    
    docker run -dit \
	   -v $CATKIN_WS_ARM:/home/pi/catkin_ws/ \
	   -v $CATKIN_WS_SRC:/home/pi/catkin_ws/src/ \
	   kmm_xc /bin/bash > ~/.xc_id
    
elif [ $1 == "stop" ] # Stop container.
then
    verify_running_container
    
    docker stop $(cat ~/.xc_id) > /dev/null
    
elif [ $1 == "make" ] # Make ARM binaries in container.
then
    verify_running_container
    
    docker exec $(cat ~/.xc_id) bash -c "source /opt/ros/kinetic/setup.bash && cd /home/pi/catkin_ws && catkin_make ${@:2}"
    
elif [ $1 == "push" ] # Push ARM binaries to robot
then
    verify_running_container
    
    tar -C $CATKIN_WS_ARM -cvf devel.tar devel
    
    MASTER_IP=$(echo $ROS_MASTER_URI | grep -o -P '(?<=http://).*(?=:)')
    scp devel.tar pi@$MASTER_IP:/home/pi/catkin_ws/
    
    ssh -t pi@$MASTER_IP "cd /home/pi/catkin_ws/ && tar -xvf devel.tar && rm devel.tar"
    
    rm devel.tar
    
else # Unknown directive
    echo "Unknown directive!"
    echo
    echo_help
fi

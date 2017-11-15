#!/usr/bin/env bash

source ~/.rosenv

CATKIN_WS_SRC=${CATKIN_WS}/src

if [ $1 == "start" ]
then
    docker run -dit \
	   -v $CATKIN_WS_ARM:/home/pi/catkin_ws/ \
	   -v $CATKIN_WS_SRC:/home/pi/catkin_ws/src/ \
	   kmm_xc /bin/bash > .id
elif [ $1 == "stop" ]
then
    docker stop $(cat .id)
    rm .id
elif [ $1 == "compile" ]
then
    docker exec $(cat .id) bash -c "source /opt/ros/kinetic/setup.bash && cd /home/pi/catkin_ws && catkin_make"
elif [ $1 == "push" ]
then
    tar -C $CATKIN_WS_ARM -cvf devel.tar devel
    
    MASTER_IP=$(echo $ROS_MASTER_URI | grep -o -P '(?<=http://).*(?=:)')
    scp devel.tar pi@$MASTER_IP:/home/pi/catkin_ws/

    ssh -t pi@$MASTER_IP "cd /home/pi/catkin_ws/ && tar -xvf devel.tar && rm devel.tar"    
    rm devel.tar
fi

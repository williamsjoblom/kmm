#!/usr/bin/env bash

scp $1 pi@$MASTER_IP:/tmp/

if [ $# -ne 2 ]
then
    echo "Usage:"
    echo "    avrdeploy <spi_port> <hex binary>"
    echo "    spi_port=0: sensor module, spi_port=1: steering module"
    echo "    ex: avrdeploy 1 stepper.hex"
    exit 1
fi

PROG_CMD="avrdude -c pi_$1 -p atmega328p -U flash:w:$2 -U lfuse:w:0xe0:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m"

MASTER_IP=$(echo $ROS_MASTER_URI | grep -o -P '(?<=http://).*(?=:)')
ssh -t pi@$MASTER_IP "cd /tmp/ && $PROG_CMD && rm $1"


#!/bin/bash
. ~/.bashrc
#sudo chmod 777 /dev/ttyACM0
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
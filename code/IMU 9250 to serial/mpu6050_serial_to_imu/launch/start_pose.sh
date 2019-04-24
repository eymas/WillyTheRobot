#!/bin/bash
while true
do
	roslaunch mpu6050_serial_to_imu/launch/demo.launch
	sleep 1
done

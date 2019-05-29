#!/bin/bash

trap 'killall' INT

killall() {
	for pid in ~/run/*.pid; do start-stop-daemon --stop --pidfile "$pid" --remove-pidfile ; done
}

start-stop-daemon --start --pidfile ~/run/roscore.pid --make-pidfile --exec /opt/ros/kinetic/bin/roscore &
sleep 5 && start-stop-daemon --start --pidfile ~/run/master_discover.pid --make-pidfile --exec /opt/ros/kinetic/bin/rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1 &
sleep 2 && start-stop-daemon --start --pidfile ~/run/master_sync.pid --make-pidfile --exec /opt/ros/kinetic/bin/rosrun master_sync_fkie master_sync &
sleep 2 && start-stop-daemon --start --pidfile ~/run/imunode.pid --make-pidfile --exec /opt/ros/kinetic/bin/rosrun mpu6050_serial_to_imu mpu6050_serial_to_imu_node &
sleep 2 && start-stop-daemon --start --pidfile ~/run/sensornode.pid --make-pidfile --exec /home/ubuntu/catkin_ws/src/sonar/src/node/start_sonar.sh &

cat

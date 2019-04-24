# pose_tracking

# Alternative 1
Install the files in the following order:

# 1
```
sudo apt-get install ros-kinetic-serial
```
# 2

Create a new workspace for the imu_tools
```
mkdir -p ~/imu_tools_ws/src
cd ~/imu_tools_ws/src
catkin_init_workspace
```
```
cd ~/imu_tools_ws
catkin_make
```
```
source devel/setup.bash
```
```
cd ~/imu_tools_ws/src
copy the files from imu_tools into the src folder of the workspace
rosdep install imu_tools
```
```
cd ~/imu_tools_ws
catkin_make
```

# 3

Create a new workspace for the imu node
```
mkdir -p ~/imu_ws/src
cd ~/imu_ws/src
catkin_init_workspace
```
```
cd ~/imu_ws
catkin_make
```
```
source devel/setup.bash
```
```
cd ~/imu_ws/src
```
copy the files from mpu6050_serial_to_imu folder into the src folder of the workspace
```
cd ~/imu_ws
catkin_make
```

# Alternavite 2


```
git clone https://github.com/Windesheim-Willy/pose_tracking.git pose_tracking
catkin_make --pkg imu_tools
catkin_make --pkg mpu6050_serial_to_imu
```
# Map USB

sudo nano /etc/udev/rules.d/99-usb-serial.rules

KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", SYMLINK+="sensor_pose"

udevadm control --reload-rules

# How to run
```
cd ~/imu_ws/src/mpu6050_serial_to_imu/launch
roslaunch demo.launch
```

#Service to reset the imu to setpoint 0.
```
rosservice call /imu/set_zero_orientation
```

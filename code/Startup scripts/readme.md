# Instructions

## Sensornode start up script
1. Ensure the following directory/file structure is present:
    ```bash
        /home/ubuntu/run/
        /home/ubuntu/sensornode.launch
        /home/ubuntu/start_sensornode.sh
    ```

2. To start the sensornode, it is necessary that the `start_sensornode.sh` is executable. Do this using `chmod +x ./start_sensornode.sh`.
3. Once the script is executable, invoke it using `./start_sensornode.sh` and the ROS Core of the sensor node will start, along with the IMU and sonar nodes.
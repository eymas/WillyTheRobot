#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <math.h>

bool zero_orientation_set = false;
const uint8_t kBytesToReceive = 46;
const float kHalfCircle = 180.0f;
const float kAdjustGyroBits = 131.0f / 65536.0f;
const float kGravity = 9.81f;
const float kAdjustAccelBits = 4.0f / 65536.0f;


//
const uint8_t kQuaternionMultFact = 100;
const uint8_t kBytesPerVar = 4;
const uint8_t kQuatWPos = 2;
const uint8_t kQuatXPos = 3;
const uint8_t kQuatYPos = 4;
const uint8_t kQuatZPos = 5;
const uint8_t kAccelXIn = 6;
const uint8_t kAccelYIn = kAccelXIn + kBytesPerVar;
const uint8_t kAccelZIn = kAccelYIn + kBytesPerVar;
const uint8_t kGyroXIn = kAccelZIn + kBytesPerVar;
const uint8_t kGyroYIn = kGyroXIn + kBytesPerVar;
const uint8_t kGyroZIn = kGyroYIn + kBytesPerVar;
const uint8_t kMagXIn = kGyroZIn + kBytesPerVar;
const uint8_t KMagYIn = kMagXIn + kBytesPerVar;
const uint8_t kMagZIN = KMagYIn + kBytesPerVar;

bool allow_store = false;
bool allow_read = false;

bool set_zero_orientation(std_srvs::Empty::Request &,
                          std_srvs::Empty::Response &) {
    ROS_INFO("Zero Orientation Set.");
    zero_orientation_set = false;
    return true;
}

union c_float {
    uint8_t input_data[kBytesPerVar];
    float output_data;
};

int main(int argc, char **argv) {
    serial::Serial ser;
    std::string port;
    std::string tf_parent_frame_id;
    std::string tf_frame_id;
    std::string frame_id;
    double time_offset_in_seconds;
    bool broadcast_tf;
    double linear_acceleration_stddev;
    double angular_velocity_stddev;
    double orientation_stddev;
    uint8_t last_received_message_number;
    bool received_message = false;
    int data_packet_start;

    tf::Quaternion orientation;
    tf::Quaternion zero_orientation;

    ros::init(argc, argv, "mpu6050_serial_to_imu_node");

    ros::NodeHandle private_node_handle("~");
    private_node_handle.param<std::string>("port", port, "/dev/ttyACM0");
    private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
    private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
    private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
    private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
    private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);
    private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
    private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
    private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);

    ros::NodeHandle nh("imu");
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 50);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("magdata", 50);
    //ros::Publisher imu_temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 50);
    ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation);

    ros::Rate r(200); // 200 hz

    sensor_msgs::Imu imu;
    sensor_msgs::MagneticField magfield;

    imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
    imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
    imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

    imu.angular_velocity_covariance[0] = angular_velocity_stddev;
    imu.angular_velocity_covariance[4] = angular_velocity_stddev;
    imu.angular_velocity_covariance[8] = angular_velocity_stddev;

    imu.orientation_covariance[0] = orientation_stddev;
    imu.orientation_covariance[4] = orientation_stddev;
    imu.orientation_covariance[8] = orientation_stddev;

    //sensor_msgs::Temperature temperature_msg;
    //temperature_msg.variance = 0;

    static tf::TransformBroadcaster tf_br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0));

    std::string input;
    std::string read;

    while (ros::ok()) {
        try {
            if (ser.isOpen()) {
                // read string from serial device
                if (ser.available()) {
                    read = ser.read(ser.available());
                    input += read;
                    //std::cout << read;
                    ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.",
                              (int) read.size(), (int) input.size());
                    while ((input.length() >=
                            kBytesToReceive)/* && allow_read*/) { // while there might be a complete package in input
                        // parse for data packets
                        std::cout << "reached while allow_read\n";
                        data_packet_start = input.find("$\x03");
                        if(data_packet_start == -1) {
                            std::cout << "start not found\r\n";
                        } else {
                            std::cout << "found start of data packet: " << data_packet_start << "\n";
                        }
                        if (data_packet_start != std::string::npos) {
                            std::cout << "found start of data packet: " << data_packet_start << "\n";
                            ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
                            if ((input.length() >= (data_packet_start + kBytesToReceive)) &&
                                (input.compare((data_packet_start + kBytesToReceive - 1), 2, "\r\n") >=
                                 0)) {  //check if positions 26,27 exist, then test values
                                std::cout << "reached processing stage" << "\n";
                                ROS_DEBUG("seems to be a real data package: long enough and found end characters");
                                // get quaternion values
                                int8_t w = (char) input[data_packet_start + kQuatWPos];
                                int8_t x = (char) input[data_packet_start + kQuatXPos];
                                int8_t y = (char) input[data_packet_start + kQuatYPos];
                                int8_t z = (char) input[data_packet_start + kQuatZPos];
                                double wf = w;
                                wf = wf / kQuaternionMultFact;
                                std::cout << "Raw: \r\n";
                                std::cout << "W: " << wf << "\r\n";
                                double xf = x;
                                xf = xf / kQuaternionMultFact;
                                std::cout << "X: " << xf << "\r\n";
                                double yf = y;
                                yf = yf / kQuaternionMultFact;
                                std::cout << "Y: " << yf << "\r\n";
                                double zf = z;
                                zf = zf / kQuaternionMultFact;
                                std::cout << "Z: " << zf << "\r\n";

                                tf::Quaternion orientation(xf, yf, zf, wf);

                                orientation = orientation.normalize();
                                std::cout << "Normalized: ";
                                std::cout << "\r\nW: " << orientation.w() << "\r\nX: " << orientation.x();
                                std::cout << "\r\nY: " << orientation.y() << "\r\nZ: " << orientation.z();
                                if (!zero_orientation_set) {
                                    zero_orientation = orientation;
                                    zero_orientation_set = true;
                                }
                                //http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
//                                tf::Quaternion differential_rotation;
//                                differential_rotation = zero_orientation.inverse() * orientation;

                                union c_float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;

                                // loop through input buffer and assign data to correct inputs
                                for (uint8_t i = 0; i < 4; i++) {
                                    accel_x.input_data[i] = static_cast<uint8_t>(input[data_packet_start + kAccelXIn +
                                                                                       i]);
                                    accel_y.input_data[i] = static_cast<uint8_t>(input[data_packet_start + kAccelYIn +
                                                                                       i]);
                                    accel_z.input_data[i] = static_cast<uint8_t>(input[data_packet_start + kAccelZIn +
                                                                                       i]);

                                    gyro_x.input_data[i] = static_cast<uint8_t>(input[data_packet_start + kGyroXIn +
                                                                                      i]);
                                    gyro_y.input_data[i] = static_cast<uint8_t>(input[data_packet_start + kGyroYIn +
                                                                                      i]);
                                    gyro_z.input_data[i] = static_cast<uint8_t>(input[data_packet_start + kGyroZIn +
                                                                                      i]);

                                    mag_x.input_data[i] = static_cast<uint8_t>(input[data_packet_start + kMagXIn + i]);
                                    mag_y.input_data[i] = static_cast<uint8_t>(input[data_packet_start + KMagYIn + i]);
                                    mag_z.input_data[i] = static_cast<uint8_t>(input[data_packet_start + kMagZIN + i]);
                                }
                                // calculate rotational velocities in rad/s
                                // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
                                // FIFO frequency 100 Hz -> factor 10 ?
                                //TODO: check / test if rotational velocities are correct

                                double gxf = gyro_x.output_data * (M_PI / kHalfCircle) * (kAdjustGyroBits);
                                double gyf = gyro_y.output_data * (M_PI / kHalfCircle) * (kAdjustGyroBits);
                                double gzf = gyro_z.output_data * (M_PI / kHalfCircle) * (kAdjustGyroBits);


                                // calculate accelerations in m/s²
                                double axf = (accel_x.output_data * kGravity * (kAdjustAccelBits));
                                double ayf = (accel_y.output_data * kGravity * (kAdjustAccelBits));
                                double azf = (accel_z.output_data * kGravity * (kAdjustAccelBits));

                                double gmx = mag_x.output_data *
                                             pow(10, -7); // convert from milligauss to Tesla for the sake of ROS
                                double gmy = mag_y.output_data * pow(10, -7);
                                double gmz = mag_z.output_data * pow(10, -7);

                                std::cout << "AXF:" << axf << " AYF:" << ayf << " AZF:" << azf << "\n";
                                std::cout << "GXF:" << gxf << " GYF:" << gyf << " GZF:" << gzf << "\n";
                                std::cout << "GMX:" << gmx << " GMY:" << gmy << " GMZ:" << gmz << "\n";
                                std::cout << "package no. " << static_cast<int>(input[data_packet_start + 42])
                                          << "\r\n";
                                uint8_t received_message_number = static_cast<int>(input[data_packet_start + 42]);

                                if (received_message) // can only check for continuous numbers if already received at least one packet
                                {
                                    uint8_t message_distance = received_message_number - last_received_message_number;
                                    if ((message_distance > 1) && message_distance != 0) {
                                        ROS_WARN_STREAM("Missed " << message_distance - 1
                                                                  << " MPU9250 data packets from arduino.");
                                    } else if (!message_distance) {
                                        ROS_WARN_STREAM("Message number is the same, could the arduino have crashed?");
                                    }
                                } else {
                                    std::cout << "received message is true. \n";
                                    received_message = true;
                                }
                                last_received_message_number = received_message_number;

                                // calculate measurement time
                                ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);
                                //std::cout << "measurement time is: " << measurement_time << "\n";

                                // publish imu message
                                imu.header.stamp = measurement_time;
                                imu.header.frame_id = frame_id;

                                magfield.header.stamp = measurement_time;
                                magfield.header.frame_id = frame_id;

                                quaternionTFToMsg(orientation, imu.orientation);


                                imu.angular_velocity.x = gxf;
                                imu.angular_velocity.y = gyf;
                                imu.angular_velocity.z = gzf;

                                imu.linear_acceleration.x = axf;
                                imu.linear_acceleration.y = ayf;
                                imu.linear_acceleration.z = azf;

                                magfield.magnetic_field.x = gmx;
                                magfield.magnetic_field.y = gmy;
                                magfield.magnetic_field.z = gmz;

                                magfield.magnetic_field_covariance[0] = 0.0025;
                                magfield.magnetic_field_covariance[4] = 0.0025;
                                magfield.magnetic_field_covariance[8] = 0.0025;

                                // set element 0 of covariance to -1 to disable measurement.
                                imu.angular_velocity_covariance[0] = -1;
                                imu.linear_acceleration_covariance[0] = -1;
                                magfield.magnetic_field_covariance[0] = -1;

                                imu_pub.publish(imu);
                                //mag_pub.publish(magfield);
//                                // publish tf transform
//                                if (broadcast_tf) {
//                                    transform.setRotation(differential_rotation);
//                                    tf_br.sendTransform(
//                                            tf::StampedTransform(transform, measurement_time, tf_parent_frame_id,
//                                                                 tf_frame_id));
//                                }
                                input.erase(0, data_packet_start + kBytesToReceive -
                                               1); // delete everything up to and including the processed packet
                            } else {
                                if (input.length() >= data_packet_start + kBytesToReceive) {
                                    std::cout << "input erase on false packet";
                                    input.erase(0, data_packet_start +
                                                   1); // delete up to false data_packet_start character so it is not found again
                                } else {
                                    std::cout << "input erase on incomplete package";
                                    // do not delete start character, maybe complete package has not arrived yet
                                    input.erase(0, data_packet_start);
                                }
                            }
                        } else {
                            std::cout << "input cleared: possibly no start character found.";
                            // no start character found in input, so delete everything
                            input.clear();
                        }
                    }
                }
            } else {
                // try and open the serial port
                try {
                    ser.setPort(port);
                    ser.setBaudrate(38400);
                    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                    ser.setTimeout(to);
                    ser.open();
                }
                catch (serial::IOException &e) {
                    ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
                    ros::Duration(5).sleep();
                }

                if (ser.isOpen()) {
                    ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
                }
            }
        }
        catch (serial::IOException &e) {
            ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
            ser.close();
        }
        ros::spinOnce();
        r.sleep();
    }

}



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
#include <time.h>

//#define PUBLISHRAW


#define Kp 2.0f * 5.0f
#define Ki 0.0f
#define PI 3.1415927f
bool zero_orientation_set = false;
const uint8_t kBytesToReceive = 42;
bool allow_store = false;
bool allow_read = false;
//=========================================================================
// vars for calculating quaternions
struct timeval tv;
struct timeval init_time;


uint32_t now = 0, lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t sumCount = 0; // control display output rate
float sum = 0.0f;
float deltat;
static float GyroMeasError = PI * (40.0f / 180.0f);
// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
static float GyroMeasDrift = PI * (0.0f  / 180.0f);
// There is a tradeoff in the beta parameter between accuracy and response
// speed. In the original Madgwick study, beta of 0.041 (corresponding to
// GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds
// to a stable initial quaternion. Subsequent changes also require a
// longish lag time to a stable output, not fast enough for a quadcopter or
// robot car! By increasing beta (GyroMeasError) by about a factor of
// fifteen, the response time constant is reduced to ~2 sec. I haven't
// noticed any reduction in solution accuracy. This is essentially the I
// coefficient in a PID control sense; the bigger the feedback coefficient,
// the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and
// fusion scheme.
static float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // Compute beta
// Compute zeta, the other free parameter in the Madgwick scheme usually
// set to a small or zero value
static float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;

// Vector to hold integral error for Mahony method
static float eInt[3] = {0.0f, 0.0f, 0.0f};
// Vector to hold quaternion
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
//=========================================================================


//
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    // short name local variable for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 +
         _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}


void updateTime() {
    gettimeofday(&tv,NULL);
    now = tv.tv_usec - init_time.tv_usec;
    // set integration time by time elapsed since last filter update
    deltat = ((now - lastUpdate) / 1000000.0f);
    lastUpdate = now;

    sum+=deltat;
    sumCount++;
}


bool set_zero_orientation(std_srvs::Empty::Request &,
                          std_srvs::Empty::Response &) {
    ROS_INFO("Zero Orientation Set.");
    zero_orientation_set = false;
    return true;
}

union c_float {
    uint8_t input_data[4];
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
    gettimeofday(&init_time,NULL);
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
                    ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.",
                              (int) read.size(), (int) input.size());
                    while ((input.length() >=
                            kBytesToReceive)/* && allow_read*/) { // while there might be a complete package in input
                        // parse for data packets
                        std::cout << "reached while allow_read\n";
                        data_packet_start = input.find("$\x03");
                        std::cout << "found start of data packet" << data_packet_start << "\n";
                        if (data_packet_start != std::string::npos) {
                            std::cout << "found start of data packet: " << data_packet_start << "\n";
                            ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
                            if ((input.length() >= (data_packet_start + kBytesToReceive)) &&
                                (input.compare((data_packet_start + kBytesToReceive - 1), 2, "\r\n") >=
                                 0)) {  //check if positions 26,27 exist, then test values
                                std::cout << "reached processing stage" << "\n";
                                ROS_DEBUG("seems to be a real data package: long enough and found end characters");
                                // get quaternion values
//                                int8_t w = (char) input[data_packet_start + 2];
//                                int8_t x = (char) input[data_packet_start + 3];
//                                int8_t y = (char) input[data_packet_start + 4];
//                                int8_t z = (char) input[data_packet_start + 5];
//                                double wf = w;
//                                wf = wf / 100;
//                                std::cout << wf << "\r\n";
//                                double xf = x;
//                                xf = xf / 100;
//                                std::cout << xf << "\r\n";
//                                double yf = y;
//                                yf = yf / 100;
//                                std::cout << yf << "\r\n";
//                                double zf = z;
//                                zf = zf / 100;
//                                std::cout << zf << "\r\n";
                                union c_float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;

                                for (uint8_t i = 0; i < 4; i++) {
                                    accel_x.input_data[i] = static_cast<uint8_t>(input[data_packet_start + 6 + i]);
                                    accel_y.input_data[i] = static_cast<uint8_t>(input[data_packet_start + 10 + i]);
                                    accel_z.input_data[i] = static_cast<uint8_t>(input[data_packet_start + 14 + i]);

                                    gyro_x.input_data[i] = static_cast<uint8_t>(input[data_packet_start + 18 + i]);
                                    gyro_y.input_data[i] = static_cast<uint8_t>(input[data_packet_start + 22 + i]);
                                    gyro_z.input_data[i] = static_cast<uint8_t>(input[data_packet_start + 26 + i]);

                                    mag_x.input_data[i] = static_cast<uint8_t>(input[data_packet_start + 30 + i]);
                                    mag_y.input_data[i] = static_cast<uint8_t>(input[data_packet_start + 34 + i]);
                                    mag_z.input_data[i] = static_cast<uint8_t>(input[data_packet_start + 38 + i]);
                                }
                                // calculate rotational velocities in rad/s
                                // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
                                // FIFO frequency 100 Hz -> factor 10 ?
                                //TODO: check / test if rotational velocities are correct
                                double gxf = gyro_x.output_data * (M_PI / 180.0) * (131.0f / 65536.0f);
                                double gyf = gyro_y.output_data * (M_PI / 180.0) * (131.0f / 65536.0f);
                                double gzf = gyro_z.output_data * (M_PI / 180.0) * (131.0f / 65536.0f);

                                // calculate accelerations in m/s²
                                double axf = (accel_x.output_data * 9.81 * (4.0f / 65536.0f));
                                double ayf = (accel_y.output_data * 9.81 * (4.0f / 65536.0f));// convert to ms^-2
                                double azf = (accel_z.output_data * 9.81 * (4.0f / 65536.0f));

                                double gmx = mag_x.output_data *
                                             pow(10, -7); // convert from milligauss to Tesla for the sake of ROS
                                double gmy = mag_y.output_data * pow(10, -7);
                                double gmz = mag_z.output_data * pow(10, -7);

                                updateTime();
                                MadgwickQuaternionUpdate(axf, ayf, azf, gxf, gyf, gzf, gmy, gmx, -gmz, deltat);
                                tf::Quaternion orientation(q[1], q[2], q[3], q[0]);

                                orientation = orientation.normalize();
                                std::cout << "X: " << orientation.x() << "Y: " << orientation.y() << "Z: " << orientation.z() << "W: " << orientation.w();
                                if (!zero_orientation_set) {
                                    zero_orientation = orientation;
                                    zero_orientation_set = true;
                                }
                                //http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
                                tf::Quaternion differential_rotation;
                                differential_rotation = zero_orientation.inverse() * orientation;



//                                std::cout << "AXF:" << axf << " AYF:" << ayf << " AZF:" << azf << "\n";
//                                std::cout << "GXF:" << gxf << " GYF:" << gyf << " GZF:" << gzf << "\n";
//                                std::cout << "GMX:" << gmx << " GMY:" << gmy << " GMZ:" << gmz << "\n";
//                                std::cout << "package no. " << static_cast<int>(input[data_packet_start + 42])
//                                          << "\r\n";
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

                                quaternionTFToMsg(differential_rotation, imu.orientation);
                                // set element 0 of covariance to -1 to disable measurement.
                                imu.angular_velocity_covariance[0] = -1;
                                imu.linear_acceleration_covariance[0] = -1;

//                                imu.angular_velocity.x = gxf;
//                                imu.angular_velocity.y = gyf;
//                                imu.angular_velocity.z = gzf;
//
//                                imu.linear_acceleration.x = axf;
//                                imu.linear_acceleration.y = ayf;
//                                imu.linear_acceleration.z = azf;
//
//                                magfield.magnetic_field.x = gmx;
//                                magfield.magnetic_field.y = gmy;
//                                magfield.magnetic_field.z = gmz;
                                magfield.magnetic_field_covariance[0] = -1;
//                              magfield.magnetic_field_covariance[0] = 0;
                                magfield.magnetic_field_covariance[4] = 0;
                                magfield.magnetic_field_covariance[8] = 0;
                                //magnetic covariance is unknown, so a 0 is sent in accordance with the MagneticField message documentation.

                                imu_pub.publish(imu);
//                            mag_pub.publish(magfield);
                                // publish tf transform
                                if (broadcast_tf) {
                                    transform.setRotation(differential_rotation);
                                    tf_br.sendTransform(
                                            tf::StampedTransform(transform, measurement_time, tf_parent_frame_id,
                                                                 tf_frame_id));
                                }
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






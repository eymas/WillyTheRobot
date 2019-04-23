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
bool allow_store = false;
bool allow_read = false;

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

union c_float {
    uint8_t input_data[4];
    float output_data;
};

int main(int argc, char** argv)
{
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
  transform.setOrigin(tf::Vector3(0,0,0));

  std::string input;
  std::string read;

  while(ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        // read string from serial device
        if(ser.available())
        {
          read = ser.read(ser.available());
          input += read;
          ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
          while ((input.length() >= kBytesToReceive)/* && allow_read*/)
          { // while there might be a complete package in input
            // parse for data packets
            std::cout << "reached while allow_read\n";
            data_packet_start = input.find("$\x03");
            std::cout << "found start of data packet" << data_packet_start << "\n";
            if (data_packet_start != std::string::npos)
            {
              std::cout << "found start of data packet: " << data_packet_start << "\n";
              ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
              if ((input.length() >= (data_packet_start + kBytesToReceive)) && (input.compare((data_packet_start + kBytesToReceive-1), 2, "\r\n") >= 0))  //check if positions 26,27 exist, then test values
              {
                std::cout << "reached processing stage" << "\n";
                ROS_DEBUG("seems to be a real data package: long enough and found end characters");
                // get quaternion values
                int8_t w = (char)input[data_packet_start + 2];
                int8_t x = (char)input[data_packet_start + 3];
                int8_t y = (char)input[data_packet_start + 4];
                int8_t z = (char)input[data_packet_start + 5];
                std::cout.precision(5);
                double wf = w;
                  wf = wf/100; std::cout << wf << "\r\n";
                double xf = x;
                  xf = xf/100; std::cout << xf << "\r\n";
                double yf = y;
                  yf = yf/100; std::cout << yf << "\r\n";
                double zf = z;
                  zf = zf/100; std::cout << zf << "\r\n";

                tf::Quaternion orientation(xf, yf, zf, wf);

                orientation.normalize();

                if (!zero_orientation_set)
                {
                  zero_orientation = orientation;
                  zero_orientation_set = true;
                }

                //http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
                tf::Quaternion differential_rotation;
                differential_rotation = zero_orientation.inverse() * orientation;

                union c_float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;

                for(uint8_t i = 0; i < 4; i++) {
                    accel_x.input_data[i] = static_cast<uint8_t>(input[data_packet_start+6+i]);
                    accel_y.input_data[i] = static_cast<uint8_t>(input[data_packet_start+10+i]);
                    accel_z.input_data[i] = static_cast<uint8_t>(input[data_packet_start+14+i]);

                    gyro_x.input_data[i] = static_cast<uint8_t>(input[data_packet_start+18+i]);
                    gyro_y.input_data[i] = static_cast<uint8_t>(input[data_packet_start+22+i]);
                    gyro_z.input_data[i] = static_cast<uint8_t>(input[data_packet_start+26+i]);

                    mag_x.input_data[i] = static_cast<uint8_t>(input[data_packet_start+30+i]);
                    mag_y.input_data[i] = static_cast<uint8_t>(input[data_packet_start+34+i]);
                    mag_z.input_data[i] = static_cast<uint8_t>(input[data_packet_start+38+i]);
                }
                // calculate rotational velocities in rad/s
                // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
                // FIFO frequency 100 Hz -> factor 10 ?
                //TODO: check / test if rotational velocities are correct
                double gxf = gyro_x.output_data * (M_PI/180.0) * (131.0f/65536.0f);
                double gyf = gyro_y.output_data * (M_PI/180.0) * (131.0f/65536.0f);
                double gzf = gyro_z.output_data * (M_PI/180.0) * (131.0f/65536.0f);

                // calculate accelerations in m/s²
                double axf = (accel_x.output_data  * 9.81 * (4.0f/65536.0f));
                double ayf = (accel_y.output_data  * 9.81 * (4.0f/65536.0f));// convert to ms^-2
                double azf = (accel_z.output_data  * 9.81 * (4.0f/65536.0f));

                double gmx = mag_x.output_data * pow(10, -7); // convert from milligauss to Tesla for the sake of ROS
                double gmy = mag_y.output_data * pow(10, -7);
                double gmz = mag_z.output_data * pow(10, -7);

                std::cout << "AXF:" << axf << " AYF:" << ayf << " AZF:" << azf << "\n";
                std::cout << "GXF:" << gxf << " GYF:" << gyf << " GZF:" << gzf << "\n";
                std::cout << "GMX:" << gmx << " GMY:" << gmy << " GMZ:" << gmz << "\n";
                std::cout << "package no. " << static_cast<int>(input[data_packet_start+42]) << "\r\n";
                uint8_t received_message_number = static_cast<int>(input[data_packet_start + 42]);

                if (received_message) // can only check for continuous numbers if already received at least one packet
                {
                  uint8_t message_distance = received_message_number - last_received_message_number;
                  if ( (message_distance > 1) && message_distance != 0 )
                  {
                    ROS_WARN_STREAM("Missed " << message_distance - 1 << " MPU9250 data packets from arduino.");
                  } else if (!message_distance) {
                      ROS_WARN_STREAM("Message number is the same, could the arduino have crashed?");
                  }
                }
                else
                {
                  std::cout << "received message is true. \n";
                  received_message = true;
                }
                last_received_message_number = received_message_number;

                // calculate measurement time
                ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);
                std::cout << "measurement time is: " << measurement_time << "\n";

                // publish imu message
                imu.header.stamp = measurement_time;
                imu.header.frame_id = frame_id;

                magfield.header.stamp = measurement_time;
                magfield.header.frame_id = frame_id;

                quaternionTFToMsg(differential_rotation, imu.orientation);

                imu.angular_velocity.x = gxf;
                imu.angular_velocity.y = gyf;
                imu.angular_velocity.z = gzf;

                imu.linear_acceleration.x = axf;
                imu.linear_acceleration.y = ayf;
                imu.linear_acceleration.z = azf;

                magfield.magnetic_field.x = gmx;
                magfield.magnetic_field.y = gmy;
                magfield.magnetic_field.z = gmz;


                //magnetic covariance is unknown, so a 0 is sent in accordance with the MagneticField message documentation.
                for(uint8_t i = 0; i < 9; i++) {
                    magfield.magnetic_field_covariance[i] = 0;
                }

                imu_pub.publish(imu);
                mag_pub.publish(magfield);

                // publish tf transform
                if (broadcast_tf)
                {
                  transform.setRotation(differential_rotation);
                  tf_br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
                }
                input.erase(0, data_packet_start + kBytesToReceive-1); // delete everything up to and including the processed packet
              }
              else
              {
                if (input.length() >= data_packet_start + kBytesToReceive)
                {
                  std::cout << "input erase on false packet";
                  input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                }
                else
                {
                  std::cout << "input erase on incomplete package";
                  // do not delete start character, maybe complete package has not arrived yet
                  input.erase(0, data_packet_start);
                }
              }
            }
            else
            {
              std::cout << "input cleared: possibly no start character found.";
              // no start character found in input, so delete everything
              input.clear();
            }
          }
        }
      }
      else
      {
        // try and open the serial port
        try
        {
          ser.setPort(port);
          ser.setBaudrate(38400);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException& e)
        {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if(ser.isOpen())
        {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
        }
      }
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    r.sleep();
  }
}



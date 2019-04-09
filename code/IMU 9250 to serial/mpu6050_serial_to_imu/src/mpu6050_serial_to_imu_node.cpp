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


bool zero_orientation_set = false;
const uint8_t kBytesToReceive = 27;

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

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
          while (input.length() >= kBytesToReceive)
          { // while there might be a complete package in input
            // parse for data packets
            data_packet_start = input.find("$3");
            if (data_packet_start != std::string::npos)
            {
                for(uint8_t i = 0; i<=kBytesToReceive; i++) {
                    std::cout << i << "." << input[data_packet_start+i] << "\r\n";
                }
              ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);

              if ((input.length() >= (data_packet_start + kBytesToReceive)) && (input.compare(data_packet_start + kBytesToReceive-1, 2, "\r\n") >= 0))  //check if positions 26,27 exist, then test values
              {
                ROS_DEBUG("seems to be a real data package: long enough and found end characters");

                // get quaternion values
                int8_t w = input[data_packet_start + 2];
                int8_t x = input[data_packet_start + 3];
                int8_t y = input[data_packet_start + 4];
                int8_t z = input[data_packet_start + 5];

                double wf = w;
                  wf = wf/100;
                double xf = x;
                  xf = xf/100;
                double yf = y;
                  yf = yf/100;
                double zf = z;
                  zf = zf/100;

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
                // get accelerometer values
                int16_t ax = ((input[data_packet_start + 6] << 8)  | input[data_packet_start + 7]);
                int16_t ay = ((input[data_packet_start + 8] << 8)  | input[data_packet_start + 9]);
                int16_t az = ((input[data_packet_start + 10] << 8) | input[data_packet_start + 11]);

                // get gyro values
                int16_t gx = ((input[data_packet_start + 12] << 8) | input[data_packet_start + 13] );
                int16_t gy = ((input[data_packet_start + 14] << 8) | input[data_packet_start + 15] );
                int16_t gz = ((input[data_packet_start + 16] << 8) | input[data_packet_start + 17] );

                // get magnetometer values
                int16_t mx = ((input[data_packet_start + 18] << 8) | input[data_packet_start + 19]);
                int16_t my = ((input[data_packet_start + 20] << 8) | input[data_packet_start + 21]);
                int16_t mz = ((input[data_packet_start + 22] << 8) | input[data_packet_start + 23]);

                // calculate rotational velocities in rad/s
                // without the last factor the velocities were too small
                // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
                // FIFO frequency 100 Hz -> factor 10 ?
                // seems 25 is the right factor
                //TODO: check / test if rotational velocities are correct
                double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

                // calculate accelerations in m/s²
                double axf = ax * (8.0 / 65536.0) * 9.81;
                double ayf = ay * (8.0 / 65536.0) * 9.81;
                double azf = az * (8.0 / 65536.0) * 9.81;

                std::cout << (int)input[data_packet_start+24];
                uint8_t received_message_number = input[data_packet_start + 24];

                if (received_message) // can only check for continuous numbers if already received at least one packet
                {
                  uint8_t message_distance = received_message_number - last_received_message_number;
                  if ( message_distance > 1 )
                  {
                    ROS_WARN_STREAM("Missed " << message_distance - 1 << " MPU9250 data packets from arduino.");
                  }
                }
                else
                {
                  received_message = true;
                }
                last_received_message_number = received_message_number;

                // calculate measurement time
                ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

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

                magfield.magnetic_field.x = mx;
                magfield.magnetic_field.y = my;
                magfield.magnetic_field.z = mz;


                //magnetic covariance is unknown, so a 0 is sent in accordance with the MagneticField message documentation.
                for(uint8_t i = 0; i < 9; i++) {
                    magfield.magnetic_field_covariance[i] = 0;
                }

                imu_pub.publish(imu);
                mag_pub.publish(magfield);

                //imu_temperature_pub.publish(temperature_msg);

                // publish tf transform
                if (broadcast_tf)
                {
                  transform.setRotation(differential_rotation);
                  tf_br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
                }
                input.erase(0, data_packet_start + kBytesToReceive); // delete everything up to and including the processed packet
              }
              else
              {
                if (input.length() >= data_packet_start + kBytesToReceive)
                {
                  input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                }
                else
                {
                  // do not delete start character, maybe complete package has not arrived yet
                  input.erase(0, data_packet_start);
                }
              }
            }
            else
            {
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



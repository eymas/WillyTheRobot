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
//const uint8_t kBytesToReceive = 27;
//const uint8_t kStorageSize = 100;
bool allow_store = false;
bool allow_read = false;

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
 // uint8_t storage[kStorageSize];
 // uint8_t storage_index = 0;
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
          input+= read;
//          uint8_t value_of_read =  static_cast<int>(read[0]);
//          if(value_of_read == '$') {
//              allow_store = true;
//              allow_read = false;
//              std::cout << "$ received";
//          }
//          if(allow_store) {
//              std::cout << "storing data";
//              int i = 0;
//              while(read[i] != '/0') {
//                  std::cout << i;
//                  storage[storage_index] = static_cast<int>(read[i]);
//                  storage_index++;
//                  if (storage_index >= kStorageSize-1) {
//                      storage_index = 0;
//                  }
//                  if (read[i] == '\n') {
//                      std::cout << "slash n received";
//                      allow_store = false;
//                      allow_read = true;
//                      storage_index = 0;
//                      break;
//                  }
//                  i++;
//              }
//          }
          ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
          while ((input.length() >= kBytesToReceive)/* && allow_read*/)
          { // while there might be a complete package in input
            // parse for data packets
            std::cout << "reached while allow_read";
            data_packet_start = input.find("$3");
            if (data_packet_start != std::string::npos)
            {
                std::cout << "found start of data packet";
              ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);

              if ((input.length() >= (data_packet_start + kBytesToReceive)) && (input.compare((data_packet_start + kBytesToReceive-1), 2, "\r\n") >= 0))  //check if positions 26,27 exist, then test values
              {
                std::cout << "reached processing stage";
                ROS_DEBUG("seems to be a real data package: long enough and found end characters");
                for(uint8_t i = 0; i <= kStorageSize; i++) {
                    std::cout << "bytes: " << storage[i];
                }
                // get quaternion values
                int8_t w = storage[data_packet_start + 2];
                int8_t x = storage[data_packet_start + 3];
                int8_t y = storage[data_packet_start + 4];
                int8_t z = storage[data_packet_start + 5];

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
                int16_t ax = ((storage[data_packet_start + 6]  << 8) | storage[data_packet_start + 9]);
                int16_t ay = ((storage[data_packet_start + 7]  << 8) | storage[data_packet_start + 10]);
                int16_t az = ((storage[data_packet_start + 8] << 8)  | storage[data_packet_start + 11]);

                // get gyro values
                int16_t gx = ((storage[data_packet_start + 12] << 8) | storage[data_packet_start + 15] );
                int16_t gy = ((storage[data_packet_start + 13] << 8) | storage[data_packet_start + 16] );
                int16_t gz = ((storage[data_packet_start + 14] << 8) | storage[data_packet_start + 17] );

                // get magnetometer values
                int16_t mx = ((storage[data_packet_start + 18] << 8) | storage[data_packet_start + 21]);
                int16_t my = ((storage[data_packet_start + 19] << 8) | storage[data_packet_start + 22]);
                int16_t mz = ((storage[data_packet_start + 20] << 8) | storage[data_packet_start + 23]);

                // calculate rotational velocities in rad/s
                // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
                // FIFO frequency 100 Hz -> factor 10 ?
                //TODO: check / test if rotational velocities are correct
                double gxf = gx * (4000.0/65536.0) * (M_PI/180.0);
                double gyf = gy * (4000.0/65536.0) * (M_PI/180.0);
                double gzf = gz * (4000.0/65536.0) * (M_PI/180.0);

                // calculate accelerations in m/s²
                double axf = ax * (8.0 / 65536.0) * 9.81;
                double ayf = ay * (8.0 / 65536.0) * 9.81;
                double azf = az * (8.0 / 65536.0) * 9.81;

                std::cout << "package no. " << storage[data_packet_start+24] << "\r\n";
                uint8_t received_message_number = storage[data_packet_start + 24];

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



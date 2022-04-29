#include "chip_imu_driver/imu_serial.h"

#include <string.h>

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace imu_sensor;

/* Private function prototypes*/
bool openConnection(imu_sensor::IMUSerial &serial); 
void publishImuMessage(ros::Publisher& imu_pub, const IMUData& data);
std::string port;
int32_t baud;

tf::Quaternion orientation_last;
ros::Time measurement_time_last;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "chip_imu_driver");
  
  //ros::NodeHandle node;
  ros::Time::init();

  /* Node */
  ros::NodeHandle node("imu");
  
  /* publisher */
  ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("data", 1);

  port = "/dev/ttyUSB0";
  baud = 115200;
  ros::NodeHandle nh("~");
  nh.param<std::string>("port", port, port);


  /* Open serial connection */
  imu_sensor::IMUSerial serial(port.c_str(), baud);
  if (openConnection(serial)) {
    ROS_INFO("Connection Succesful");
  } else {
    ROS_WARN("Problem connecting to serial device (number of attempts is 100)");
    return 1;
  }

  /* Main cycle */
  ros::Rate r(500); // 500 hz
  while (ros::ok())
  {
    serial.readAndParse();
    if (serial.hasNewData()) {
      publishImuMessage(imu_pub, serial.getData());
    }
    ros::spinOnce();
    r.sleep();
  }
}


bool openConnection(imu_sensor::IMUSerial &serial) 
{
  int n_attempts = 100;
  ros::Rate r(10); // Hz
  while (n_attempts != 0) {
    ROS_INFO("Attempting connection to %s at %i baud.", port.c_str(), baud);

    if (serial.connect()) {
      return true;
    }
    n_attempts--;
    ros::spinOnce();
    r.sleep();
  }

  return false;
}

/**
 * @brief Publish imu message 
 */
void publishImuMessage(ros::Publisher& imu_pub, const IMUData& data)
{
  double linear_acceleration_stddev = 0;
  double angular_velocity_stddev = 0;
  double orientation_stddev = 0;
  string frame_id = "base_link";

  // calculate measurement time
  ros::Time measurement_time = ros::Time::now(); // + ros::Duration(time_offset_in_seconds);

  sensor_msgs::Imu imu;
  imu.header.stamp = measurement_time;
  imu.header.frame_id = frame_id;

  /* covariance matrices */
  imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

  imu.angular_velocity_covariance[0] = angular_velocity_stddev;
  imu.angular_velocity_covariance[4] = angular_velocity_stddev;
  imu.angular_velocity_covariance[8] = angular_velocity_stddev;

  imu.orientation_covariance[0] = orientation_stddev;
  imu.orientation_covariance[4] = orientation_stddev;
  imu.orientation_covariance[8] = orientation_stddev;
 
  /* acceleration */
  imu.linear_acceleration.x = data.x_acc;
  imu.linear_acceleration.y = data.y_acc;
  imu.linear_acceleration.z = data.z_acc;

  /* Orientation */
  tf::Quaternion orientation;
  orientation.setRPY(data.roll*M_PI/180, data.pitch*M_PI/180, data.yaw*M_PI/180);

  // Convert Quaternion to Quaternion msg
  tf::quaternionTFToMsg(orientation, imu.orientation);
  
  /* angular velocity */
  tf::Quaternion orientation_diff = orientation * (orientation_last.inverse());
  double measurement_time_diff = (measurement_time - measurement_time_last).toSec();
  tf::Matrix3x3 mat(orientation_diff);
  tfScalar yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  imu.angular_velocity.x = roll / measurement_time_diff;
  imu.angular_velocity.y = pitch / measurement_time_diff;
  imu.angular_velocity.z = yaw / measurement_time_diff;

  imu_pub.publish(imu);

  measurement_time_last = measurement_time;
  orientation_last = orientation;
}

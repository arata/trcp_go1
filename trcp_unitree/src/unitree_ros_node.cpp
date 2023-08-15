#include <iostream>
#include <chrono>
#include <pthread.h>

#include <ros/ros.h>

// tf library
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

// ros msg
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

// unitree lib
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/MotorState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  UDP low_udp;
  UDP high_udp;
  
  HighCmd high_cmd = {0};
  HighState high_state = {0};
  
  LowCmd low_cmd = {0};
  LowState low_state = {0};

public:
  Custom()
    : 
    // low_udp(LOWLEVEL),
    low_udp(LOWLEVEL, 8091, "192.168.123.10", 8007),
    high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
  {
    high_udp.InitCmdData(high_cmd);
    low_udp.InitCmdData(low_cmd);
  }

  /* highUdp */
  void highUdpSend() {
    high_udp.SetSend(high_cmd);
    high_udp.Send();
  }

  void highUdpRecv() {
    high_udp.Recv();
    high_udp.GetRecv(high_state);
  }

  /* lowUdp */
  void lowUdpSend() {
    low_udp.SetSend(low_cmd);
    low_udp.Send();
  }

  void lowUdpRecv() {
    low_udp.Recv();
    low_udp.GetRecv(low_state);
  }
};

class UnitreeRos
{
public:
  UnitreeRos(Custom *cust) : nh_()
  {
    std::cout << "start unitree ros interface" << std::endl;

    cus = cust;

    // publisher timer
    rec_data = nh_.createTimer(ros::Duration(0.1), &UnitreeRos::receiveDataFromRobot, this);
    pub_odom_timer = nh_.createTimer(ros::Duration(0.1), &UnitreeRos::publishOdometry, this);
    pub_joint_state_timer = nh_.createTimer(ros::Duration(0.1), &UnitreeRos::publishJointState, this);
    pub_imu_timer = nh_.createTimer(ros::Duration(0.1), &UnitreeRos::publishImu, this);    

    pub_high = nh_.advertise<unitree_legged_msgs::HighState>("high_state", 1, this);
    pub_low = nh_.advertise<unitree_legged_msgs::LowState>("low_state", 1, this);
    pub_odom = nh_.advertise<nav_msgs::Odometry>("odom", 1, this);
    pub_joint_state = nh_.advertise<sensor_msgs::JointState>("joint_states", 1, this);
    pub_imu = nh_.advertise<sensor_msgs::Imu>("imu_raw", 1, this);

  }

private:

  ros::NodeHandle nh_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster dynamic_br_;

  // timer
  ros::Timer rec_data;
  ros::Timer pub_odom_timer;
  ros::Timer pub_joint_state_timer;
  ros::Timer pub_imu_timer;

  // subscriber
  ros::Subscriber sub_cmd_vel;
  
  // publisher
  ros::Publisher pub_odom;
  ros::Publisher pub_high;
  ros::Publisher pub_low;
  ros::Publisher pub_joint_state;
  ros::Publisher pub_imu;

  unitree_legged_msgs::HighState high_state_ros;
  unitree_legged_msgs::LowState low_state_ros;

  float old_speed = 0;
  float difference = 0;  

  Custom *cus;

  void publishOdometry(const ros::TimerEvent& e)
  {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.seq = 0;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = high_state_ros.position[0];
    odom_msg.pose.pose.position.y = high_state_ros.position[1];
    odom_msg.pose.pose.position.z = high_state_ros.position[2];

    float timespan = 0.1;
    float speed = high_state_ros.yawSpeed;
    difference = ((speed + old_speed) * timespan) / 2 + difference;

    old_speed = speed;

    // std::cout << odom_msg.pose.pose.position.x << ", "
    //           << odom_msg.pose.pose.position.y << ", "
    //           << odom_msg.pose.pose.position.z << std::endl;

    // std::cout << difference << std::endl;

    tf2::Quaternion q;
    q.setRPY(0, 0, difference);

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // std::cout << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;

    pub_odom.publish(odom_msg);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = odom_msg.header.stamp;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = odom_msg.pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
    transformStamped.transform.translation.z = odom_msg.pose.pose.position.z;
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // dynamic_br_.sendTransform(transformStamped);

    // std::cout << "send tf" << std::endl;
  }

  void publishJointState(const ros::TimerEvent& e)
  {
    boost::array<unitree_legged_msgs::MotorState, 20> motorState = high_state_ros.motorState;

    sensor_msgs::JointState jointState;

    std::vector<std::string> leg_joint_name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                               "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                                               "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                                               "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

    jointState.header.stamp = ros::Time::now();
    for(int i=0; i < 12; i++) { // 12 is number of motor
      jointState.name.push_back(leg_joint_name[i]);
      jointState.position.push_back(motorState[i].q);
      jointState.velocity.push_back(motorState[i].dq);
      jointState.effort.push_back(motorState[i].tauEst);
    }

    pub_joint_state.publish(jointState);    
  }

  void publishImu(const ros::TimerEvent& e)
  {
    sensor_msgs::Imu imu;

    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "imu_link";

    imu.orientation.x = high_state_ros.imu.quaternion[1];
    imu.orientation.y = high_state_ros.imu.quaternion[2];
    imu.orientation.z = high_state_ros.imu.quaternion[3];
    imu.orientation.w = high_state_ros.imu.quaternion[0];
    
    imu.angular_velocity.x = high_state_ros.imu.gyroscope[0];
    imu.angular_velocity.y = high_state_ros.imu.gyroscope[1];
    imu.angular_velocity.z = high_state_ros.imu.gyroscope[2];

    imu.linear_acceleration.x = high_state_ros.imu.accelerometer[0];
    imu.linear_acceleration.y = high_state_ros.imu.accelerometer[1];
    imu.linear_acceleration.z = high_state_ros.imu.accelerometer[2];
      
    pub_imu.publish(imu);

    std::cout << "roll  : " << high_state_ros.imu.rpy[0] << std::endl;
    std::cout << "pitch : " << high_state_ros.imu.rpy[1] << std::endl;
    std::cout << "yaw   : " << high_state_ros.imu.rpy[2] << std::endl;
    std::cout << "---------------" << std::endl;
  }

  void receiveDataFromRobot(const ros::TimerEvent& e)
  {
    // high_state_ros = state2rosMsg(custom.high_state);
    high_state_ros = state2rosMsg(cus->high_state);
    // low_state_ros = state2rosMsg(custom.low_state);

    pub_high.publish(high_state_ros);
    // pub_low.publish(low_state_ros);
  }

};

int main(int argc, char **argv) {

  std::cout << "start trcp" << std::endl;

  ros::init(argc, argv, "unitree_base_controller");

  Custom custom;

  LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
  LoopFunc loop_udpRecvHigh("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));
  LoopFunc loop_udpRecvLow("low_udp_recv", 0.002, 3, boost::bind(&Custom::lowUdpRecv, &custom));

  loop_udpSend.start();
  loop_udpRecvHigh.start();
  loop_udpRecvLow.start();

  UnitreeRos unitree_ros(&custom);

  ros::spin();

  return 0;
}

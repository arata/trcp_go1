#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/MotorState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

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

  /*
    highUdp
  */
  void highUdpSend() {
    high_udp.SetSend(high_cmd);
    high_udp.Send();
  }

  void highUdpRecv() {
    high_udp.Recv();
    high_udp.GetRecv(high_state);
  }

  /*
    lowUdp
  */
  void lowUdpSend() {
    low_udp.SetSend(low_cmd);
    low_udp.Send();
  }

  void lowUdpRecv() {
    low_udp.Recv();
    low_udp.GetRecv(low_state);
  }
};

Custom custom;

// ros definition

ros::Subscriber sub_cmd_vel;
ros::Publisher pub_odom;
ros::Publisher pub_high;
ros::Publisher pub_low;
ros::Publisher pub_joint_state;

// ROS state of the robot
unitree_legged_msgs::HighState high_state_ros;
unitree_legged_msgs::LowState low_state_ros;

float old_speed = 0;
float difference = 0;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{

  custom.high_cmd = rosMsg2Cmd(msg);

}

void publishOdometry(const ros::TimerEvent& e)
{
  // tf2_ros::TransformBroadcaster dynamic_br_;

  nav_msgs::Odometry odom_msg;
  odom_msg.header.seq = 0;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base";
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

  //std::cout << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;

  // pub_odom.publish(odom_msg);

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = odom_msg.header.stamp;
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base";
  transformStamped.transform.translation.x = odom_msg.pose.pose.position.x;
  transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
  // transformStamped.transform.translation.z = odom_msg.pose.pose.position.z;
  transformStamped.transform.translation.z = 0;  
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  tf2_ros::TransformBroadcaster dynamic_br_;  
  dynamic_br_.sendTransform(transformStamped);

  std::cout << "send tf" << std::endl;
  
}

void publishJointState(const ros::TimerEvent& e){

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

void receiveDataFromRobot(const ros::TimerEvent& e)
{

  high_state_ros = state2rosMsg(custom.high_state);
  // low_state_ros = state2rosMsg(custom.low_state);

  pub_high.publish(high_state_ros);
  // pub_low.publish(low_state_ros);

}

int main(int argc, char **argv)
{

  std::cout << "start trcp" << std::endl;

  ros::init(argc, argv, "unitree_base_controller");

  ros::NodeHandle nh;

  LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
  LoopFunc loop_udpRecvHigh("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));
  LoopFunc loop_udpRecvLow("low_udp_recv", 0.002, 3, boost::bind(&Custom::lowUdpRecv, &custom));

  loop_udpSend.start();
  loop_udpRecvHigh.start();
  loop_udpRecvLow.start();

  // tf 
  // tf2_ros::TransformBroadcaster dynamic_br_;

  // subscriber
  sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);

  // publisher
  pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
  pub_low = nh.advertise<unitree_legged_msgs::LowState>("low_state", 1);  
  pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);
  pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  // timer
  ros::Timer rec_data = nh.createTimer(ros::Duration(0.1), receiveDataFromRobot);
  ros::Timer pub_odom_timer = nh.createTimer(ros::Duration(0.1), publishOdometry);
  ros::Timer pub_joint_state_timer = nh.createTimer(ros::Duration(0.1), publishJointState);
  
  ros::spin();

  return 0;
}

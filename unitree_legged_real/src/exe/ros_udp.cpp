#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/IMU.h>
#include <unitree_legged_msgs/MotorState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>

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

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

Custom custom;

// Subscribers
ros::Subscriber sub_high;
ros::Subscriber sub_low;

// Publishers
ros::Publisher pub_high;
ros::Publisher pub_low;
ros::Publisher pub_imu;
ros::Publisher pub_odom;
ros::Publisher pub_jointfoot;

long high_count = 0;
long low_count = 0;

void highCmdCallback(const unitree_legged_msgs::HighCmd::ConstPtr &msg)
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    unitree_legged_msgs::HighState high_state_ros;
    unitree_legged_msgs::IMU unitree_imu_msg;
    unitree_legged_msgs::MotorState unitree_joint_msg;

    sensor_msgs::Imu imu_msg;
    sensor_msgs::JointState joint_foot_msg;
    nav_msgs::Odometry odom_msg;

    high_state_ros = state2rosMsg(custom.high_state);
    unitree_imu_msg = state2rosMsg(custom.high_state.imu);

    // Conver unitree_imu to sensor msg imu
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.orientation.x = unitree_imu_msg.quaternion[1];
    imu_msg.orientation.y = unitree_imu_msg.quaternion[2];
    imu_msg.orientation.z = unitree_imu_msg.quaternion[3];
    imu_msg.orientation.w = unitree_imu_msg.quaternion[0];

    imu_msg.angular_velocity.x = unitree_imu_msg.gyroscope[0];
    imu_msg.angular_velocity.y = unitree_imu_msg.gyroscope[1];
    imu_msg.angular_velocity.z = unitree_imu_msg.gyroscope[2];

    imu_msg.linear_acceleration.x = unitree_imu_msg.accelerometer[0];
    imu_msg.linear_acceleration.y = unitree_imu_msg.accelerometer[1];
    imu_msg.linear_acceleration.z = unitree_imu_msg.accelerometer[2]; 

    // convert unitree motor state to sensor msg joint state
    std::vector<std::string> joint_names = 
    {
        "FL0", "FL1", "FL2", "FR0", "FR1", "FR2",
        "RL0", "RL1", "RL2", "RR0", "RR1", "RR2",
        "FL_foot", "FR_foot", "RL_foot", "RR_foot"
    };

    joint_foot_msg.header.stamp = ros::Time::now();;

    for (int i = 0; i < 16; ++i) 
    {   
        unitree_joint_msg = state2rosMsg(custom.high_state.motorState[i]);
        joint_foot_msg.name.push_back(joint_names[i]);
        joint_foot_msg.position.push_back(unitree_joint_msg.q);
        joint_foot_msg.velocity.push_back(unitree_joint_msg.dq);
        joint_foot_msg.effort.push_back(unitree_joint_msg.tauEst);
    }

    // convert unitree high state for position and unitree imu quat for orientation
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = high_state_ros.position[0]; 
    odom_msg.pose.pose.position.y = high_state_ros.position[1];
    odom_msg.pose.pose.position.z = high_state_ros.position[2];
    odom_msg.pose.pose.orientation.x =  unitree_imu_msg.quaternion[1];
    odom_msg.pose.pose.orientation.y =  unitree_imu_msg.quaternion[2];
    odom_msg.pose.pose.orientation.z =  unitree_imu_msg.quaternion[3];
    odom_msg.pose.pose.orientation.w =  unitree_imu_msg.quaternion[0];

    pub_high.publish(high_state_ros);
    pub_imu.publish(imu_msg);
    pub_jointfoot.publish(joint_foot_msg);
    pub_odom.publish(odom_msg);;

    printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

void lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{

    printf("lowCmdCallback is running !\t%ld\n", low_count);

    custom.low_cmd = rosMsg2Cmd(msg);

    unitree_legged_msgs::LowState low_state_ros;

    low_state_ros = state2rosMsg(custom.low_state);

    pub_low.publish(low_state_ros);

    printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_udp");

    ros::NodeHandle nh;

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        sub_low = nh.subscribe("low_cmd", 1, lowCmdCallback);
        pub_low = nh.advertise<unitree_legged_msgs::LowState>("low_state", 1);

        LoopFunc loop_udpSend("low_udp_send", 0.002, 3, boost::bind(&Custom::lowUdpSend, &custom));
        LoopFunc loop_udpRecv("low_udp_recv", 0.002, 3, boost::bind(&Custom::lowUdpRecv, &custom));

        loop_udpSend.start();
        loop_udpRecv.start();

        printf("LOWLEVEL is initialized\n");

        ros::spin();

        // printf("low level runing!\n");
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        sub_high = nh.subscribe("high_cmd", 1, highCmdCallback);

        pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
        pub_imu = nh.advertise<sensor_msgs::Imu>("hardware_go1/imu", 1);
        pub_jointfoot = nh.advertise<sensor_msgs::JointState>("/hardware_go1/joint_foot", 1);
        pub_odom = nh.advertise<nav_msgs::Odometry>("/hardware_go1/estimated_odom", 1);

        LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
        LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

        loop_udpSend.start();
        loop_udpRecv.start();

        printf("HIGHLEVEL is initialized\n");
        
        ros::spin();

        // printf("high level runing!\n");
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    return 0;
}

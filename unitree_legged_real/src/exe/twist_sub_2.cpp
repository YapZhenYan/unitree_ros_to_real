#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

using namespace UNITREE_LEGGED_SDK;
// #define DEBUG

HighCmd high_cmd = {0};
HighState high_state = {0};

unitree_legged_msgs::HighCmd new_high_cmd;
unitree_legged_msgs::HighState high_state_ros;

ros::Publisher pub_high;
ros::Publisher pub_imu;
ros::Publisher pub_jointfoot;
ros::Publisher pub_odom;

long cmd_vel_count = 0;
double x_prev = 0;
double y_prev = 0;
double yaw_prev = 0;
double x_curr;
double y_curr;
double yaw_curr;

float constrain(float val, float max, float min){
    if (val > max)
        return max;
    else if (val < min)
        return min;
    else
        return val;
}

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &state)
{
    #ifdef DEBUG
        ROS_INFO("highStateCallback %ld", count++);
    #endif
    // high_state_ros = *state;
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    sensor_msgs::Imu imu_msg;
    // Extract IMU data
    imu_msg.header = header;
    imu_msg.orientation.x = state->imu.quaternion[1];
    imu_msg.orientation.y = state->imu.quaternion[2];
    imu_msg.orientation.z = state->imu.quaternion[3];
    imu_msg.orientation.w = state->imu.quaternion[0];

    imu_msg.angular_velocity.x = state->imu.gyroscope[0];
    imu_msg.angular_velocity.y = state->imu.gyroscope[1];
    imu_msg.angular_velocity.z = state->imu.gyroscope[2];

    imu_msg.linear_acceleration.x = state->imu.accelerometer[0];
    imu_msg.linear_acceleration.y = state->imu.accelerometer[1];
    imu_msg.linear_acceleration.z = state->imu.accelerometer[2];    


    // Extract Odom data
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = state->position[0]; 
    odom_msg.pose.pose.position.y = state->position[1];
    odom_msg.pose.pose.position.z = state->position[2];
    odom_msg.pose.pose.orientation.x =  state->imu.quaternion[1];
    odom_msg.pose.pose.orientation.y =  state->imu.quaternion[2];
    odom_msg.pose.pose.orientation.z =  state->imu.quaternion[3];
    odom_msg.pose.pose.orientation.w =  state->imu.quaternion[0];

    sensor_msgs::JointState joint_foot_msg;
    std::vector<std::string> joint_names = 
    {
        "FL0", "FL1", "FL2", "FR0", "FR1", "FR2",
        "RL0", "RL1", "RL2", "RR0", "RR1", "RR2",
        "FL_foot", "FR_foot", "RL_foot"
    };

    joint_foot_msg.header = header;

    // Extract motor states and populate the JointState message
    for (int i = 0; i < 12; ++i) {

        // Extract motor state data for each leg
        const unitree_legged_msgs::MotorState& motor_state = state->motorState[i];

         // Assuming you want to populate position, velocity, and effort fields
        joint_foot_msg.name.push_back(joint_names[i]);
        joint_foot_msg.position.push_back(motor_state.q);
        joint_foot_msg.velocity.push_back(motor_state.dq);
        joint_foot_msg.effort.push_back(motor_state.tauEst);
    }


    pub_imu.publish(imu_msg);
    pub_jointfoot.publish(joint_foot_msg);
    pub_odom.publish(odom_msg);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{   

    high_cmd = rosMsg2Cmd(msg);

    // setting speed limit
    high_cmd.velocity[0] = constrain(high_cmd.velocity[0] , 0.4, -0.4);
    high_cmd.velocity[1] = constrain(high_cmd.velocity[1] , 0.4,-0.4);
    high_cmd.yawSpeed = constrain(high_cmd.yawSpeed, 0.5, -0.5); 

    #ifdef DEBUG
        printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);
        printf("cmd_x_vel = %f\n", high_cmd.velocity[0]);
        printf("cmd_y_vel = %f\n", high_cmd.velocity[1]);
        printf("cmd_yaw_vel = %f\n", high_cmd.yawSpeed);
    #endif
    
    x_curr = high_cmd.velocity[0];
    y_curr = high_cmd.velocity[1];
    yaw_curr = high_cmd.yawSpeed;
    
    new_high_cmd.velocity[0] = x_curr * 0.8 + x_prev * 0.2;
    new_high_cmd.velocity[1] = y_curr * 0.8 + y_prev * 0.2;
    new_high_cmd.yawSpeed = yaw_curr * 0.8 + yaw_prev * 0.2;

    x_prev = x_curr;
    y_prev = y_curr;
    yaw_prev = yaw_curr;

    high_state_ros = state2rosMsg(high_state);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_sub_2");

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;

    ros::Rate loop_rate(500);

    long motiontime = 0;

    unitree_legged_msgs::HighCmd high_cmd_ros;
    unitree_legged_msgs::HighState high_state_ros;

    // Publisher
    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1);
    pub_imu = nh.advertise<sensor_msgs::Imu>("/hardware_go1/imu", 1);
    pub_jointfoot = nh.advertise<sensor_msgs::JointState>("/hardware_go1/joint_foot", 1);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/hardware_go1/estimated_odom", 1);

    // Subscriber
    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);
    ros::Subscriber sub = nh.subscribe("high_state", 1, highStateCallback);

    while (ros::ok())
    {   
        // printf("rpy: %f %f %f\n", high_state_ros.imu.rpy[0], high_state_ros.imu.rpy[1], high_state_ros.imu.rpy[2]);
        
        motiontime += 2;

        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;
        // high_cmd_ros.speedLevel = 0;
        // high_cmd_ros.footRaiseHeight = 0;
        // high_cmd_ros.bodyHeight = 0;
        // high_cmd_ros.euler[0] = 0;
        // high_cmd_ros.euler[1] = 0;
        // high_cmd_ros.euler[2] = 0;
        // high_cmd_ros.reserve = 0;

    
        if (motiontime < 4)
        {   
            high_cmd_ros.mode = 0;
            high_cmd_ros.gaitType = 0;
            high_cmd_ros.velocity[0] = 0.0f;
            high_cmd_ros.velocity[1] = 0.0f;
            high_cmd_ros.yawSpeed = 0.0f;
        }

        if (motiontime >=4)
        {
            // walk
            high_cmd_ros.mode = 2;
            high_cmd_ros.gaitType = 1;
            high_cmd_ros.velocity[0] = new_high_cmd.velocity[0];
            high_cmd_ros.velocity[1] = new_high_cmd.velocity[1];
            high_cmd_ros.yawSpeed =  new_high_cmd.yawSpeed; 
            high_cmd_ros.footRaiseHeight = 0.08;
        }
        pub.publish(high_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;
// #define DEBUG

HighCmd high_cmd = {0};
HighState high_state = {0};

unitree_legged_msgs::HighCmd new_high_cmd;
unitree_legged_msgs::HighState high_state_ros;

ros::Publisher pub_high;

long cmd_vel_count = 0;
double x_prev = 0;
double y_prev = 0;
double yaw_prev = 0;
double x_curr;
double y_curr;
double yaw_curr;

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &state)
{
    static long count = 0;
    #ifdef DEBUG
        ROS_INFO("highStateCallback %ld", count++);
    #endif
    high_state_ros = *state;
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{   


    high_cmd = rosMsg2Cmd(msg);
    
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

    pub_high.publish(high_state_ros);

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
    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);
    pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);

    // Subscriber
    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);
    ros::Subscriber sub = nh.subscribe("high_state", 1000, highStateCallback);

    
    while (ros::ok())
    {   
        // printf("rpy: %f %f %f\n", high_state_ros.imu.rpy[0], high_state_ros.imu.rpy[1], high_state_ros.imu.rpy[2]);
        
        motiontime += 2;

        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;
        high_cmd_ros.mode = 0;
        high_cmd_ros.gaitType = 0;
        high_cmd_ros.speedLevel = 0;
        high_cmd_ros.footRaiseHeight = 0;
        high_cmd_ros.bodyHeight = 0;
        high_cmd_ros.euler[0] = 0;
        high_cmd_ros.euler[1] = 0;
        high_cmd_ros.euler[2] = 0;
        high_cmd_ros.velocity[0] = 0.0f;
        high_cmd_ros.velocity[1] = 0.0f;
        high_cmd_ros.yawSpeed = 0.0f;
        high_cmd_ros.reserve = 0;

        if (motiontime >=4)
        {
            // walk
            high_cmd_ros.mode = 2;
            high_cmd_ros.gaitType = 1;
            high_cmd_ros.velocity[0] = new_high_cmd.velocity[0];
            high_cmd_ros.velocity[1] = new_high_cmd.velocity[1];
            high_cmd_ros.yawSpeed =  new_high_cmd.yawSpeed; 
            high_cmd_ros.footRaiseHeight = 0.1;
        }
        pub.publish(high_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



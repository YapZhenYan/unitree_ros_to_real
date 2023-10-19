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
// HighState high_state = {0};

unitree_legged_msgs::HighCmd new_high_cmd;
// unitree_legged_msgs::HighState high_state_ros;
unitree_legged_msgs::HighCmd high_cmd_ros;

// ros::Publisher pub_high;
ros::Publisher pub;

long cmd_vel_count = 0;
double x_prev = 0;
double y_prev = 0;
double yaw_prev = 0;
double x_curr;
double y_curr;
double yaw_curr;

long motiontime = 0;


float constrain(float val, float max, float min){
    if (val > max)
        return max;
    else if (val < min)
        return min;
    else
        return val;
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

    // high_state_ros = state2rosMsg(high_state);

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

    // Publisher
    pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1);

    // Subscriber
    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);
    // ros::Subscriber sub = nh.subscribe("high_state", 1, highStateCallback);

    while (ros::ok())
    {   
        
        motiontime += 2;

        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;

    
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


    // ros::AsyncSpinner spinner(4); // Use 4 threads
    // spinner.start();
    // ros::waitForShutdown();
    return 0;
}
#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

unitree_legged_msgs::HighState high_state_ros;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

    unitree_legged_msgs::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high.publish(high_state_ros);

    printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &state)
{
    static long count = 0;
    ROS_INFO("highStateCallback %ld", count++);
    high_state_ros = *state;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_walk_without_lcm");

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;

    ros::Rate loop_rate(500);

    long motiontime = 0;

    unitree_legged_msgs::HighCmd high_cmd_ros;

    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);
    ros::Subscriber sub = nh.subscribe("high_state", 1000, highStateCallback);
    while (ros::ok())
    {   
        printf("rpy: %f %f %f\n", high_state_ros.imu.rpy[0], high_state_ros.imu.rpy[1], high_state_ros.imu.rpy[2]);
        
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

        // walk
        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 2;
        high_cmd_ros.velocity[0] = /* cmd_vel from ros topic */// 0.4f; // -1  ~ +1
        high_cmd_ros.velocity[1] = ; // y
        high_cmd_ros.velocity[2] = ; // z
        high_cmd_ros.yawSpeed =  /* cmd_vel from ros topic */;
        high_cmd_ros.footRaiseHeight = 0.1;
        // printf("walk\n");

        // if (motiontime > 0 && motiontime < 1000)
        // {
        //     high_cmd_ros.mode = 1;
        //     high_cmd_ros.euler[0] = -0.3;
        // }
        // if (motiontime > 1000 && motiontime < 2000)
        // {
        //     high_cmd_ros.mode = 1;
        //     high_cmd_ros.euler[0] = 0.3;
        // }
        // if (motiontime > 2000 && motiontime < 3000)
        // {
        //     high_cmd_ros.mode = 1;
        //     high_cmd_ros.euler[1] = -0.2;
        // }
        // if (motiontime > 3000 && motiontime < 4000)
        // {
        //     high_cmd_ros.mode = 1;
        //     high_cmd_ros.euler[1] = 0.2;
        // }
        // if (motiontime > 4000 && motiontime < 5000)
        // {
        //     high_cmd_ros.mode = 1;
        //     high_cmd_ros.euler[2] = -0.2;
        // }
        // if (motiontime > 5000 && motiontime < 6000)
        // {
        //     high_cmd_ros.mode = 1;
        //     high_cmd_ros.euler[2] = 0.2;
        // }
        // if (motiontime > 6000 && motiontime < 7000)
        // {
        //     high_cmd_ros.mode = 1;
        //     high_cmd_ros.bodyHeight = -0.2;
        // }
        // if (motiontime > 7000 && motiontime < 8000)
        // {
        //     high_cmd_ros.mode = 1;
        //     high_cmd_ros.bodyHeight = 0.1;
        // }
        // if (motiontime > 8000 && motiontime < 9000)
        // {
        //     high_cmd_ros.mode = 1;
        //     high_cmd_ros.bodyHeight = 0.0;
        // }
        // if (motiontime > 9000 && motiontime < 11000)
        // {
        //     high_cmd_ros.mode = 5;
        // }
        // if (motiontime > 11000 && motiontime < 13000)
        // {
        //     high_cmd_ros.mode = 6;
        // }
        // if (motiontime > 13000 && motiontime < 14000)
        // {
        //     high_cmd_ros.mode = 0;
        // }
        // if (motiontime > 14000 && motiontime < 18000)
        // {
        //     high_cmd_ros.mode = 2;
        //     high_cmd_ros.gaitType = 2;
        //     high_cmd_ros.velocity[0] = 0.4f; // -1  ~ +1
        //     high_cmd_ros.yawSpeed = 2;
        //     high_cmd_ros.footRaiseHeight = 0.1;
        //     // printf("walk\n");
        // }
        // if (motiontime > 18000 && motiontime < 20000)
        // {
        //     high_cmd_ros.mode = 0;
        //     high_cmd_ros.velocity[0] = 0;
        // }
        // if (motiontime > 20000 && motiontime < 24000)
        // {
        //     high_cmd_ros.mode = 2;
        //     high_cmd_ros.gaitType = 1;
        //     high_cmd_ros.velocity[0] = 0.2f; // -1  ~ +1
        //     high_cmd_ros.bodyHeight = 0.1;
        //     // printf("walk\n");
        // }
        // if (motiontime > 24000)
        // {
        //     high_cmd_ros.mode = 1;
        // }
        pub.publish(high_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_sub");

    ros::NodeHandle nh;

    pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);

    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();

    ros::spin();

    return 0;
}

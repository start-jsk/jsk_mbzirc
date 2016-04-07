/*
 * jsk MBZIRC
 */

// Author: author: chen

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

class TeleopUAVJoy
{
private:
    double walk_vel, run_vel, yaw_rate, yaw_rate_run, vertical_vel;
    geometry_msgs::Twist cmd;
    std_msgs::Float64 gripper;
    ros::NodeHandle n_;
    ros::Publisher vel_pub_;
    ros::Publisher grip_pub_;
    ros::Subscriber joy_sub_;

    void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
public:
    void init()
    {
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
        gripper.data = 0;
        joy_sub_ = n_.subscribe<sensor_msgs::Joy>("/teleop_joy",10,&TeleopUAVJoy::JoyCallback,this);
        vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        grip_pub_ = n_.advertise<std_msgs::Float64>("/r_gripper_controller/command",1);
        if(!n_.getParam("teleopUGV",teleopUGV))
            puts("fail to load the param");
        ros::NodeHandle n_private("~");
        n_private.param("walk_vel", walk_vel, 1.0);
        n_private.param("run_vel", run_vel, 8.0);
        n_private.param("yaw_rate", yaw_rate, 0.5);
        n_private.param("yaw_run_rate", yaw_rate_run, 1.5);
        n_private.param("vertical_vel", vertical_vel, 2.0);
    }
    bool teleopUGV=false;
    ~TeleopUAVJoy()   { }
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jsk_mbzirc_joy");

    TeleopUAVJoy tpj;
    tpj.init();

    signal(SIGINT,quit);
    puts("Reading from joy");
    puts("---------------------------");
    if(tpj.teleopUGV)
    {
        puts("Hold 'L1' for nomal movement");
        puts("Use 'left axis' to horizontal translate");
        puts("Use 'right axis' to yaw and up/down");
        puts("Hold 'R1' for fast movement");
        puts("press 'O and X' to open and close the gripper");
    }
    else
    {
        puts("Hold 'L1' for nomal movement");
        puts("Use 'left axis' to horizontal translate");
        puts("Use 'right axis' to yaw and up/down");
        puts("Hold 'R1' for fast movement");
    }
    ros::spin();

    return(0);
}

void TeleopUAVJoy::JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    bool dirty=false;
    bool dirtygripper=false;

    cmd.linear.x = cmd.linear.y = cmd.angular.z = cmd.linear.z = 0;

    if(joy->buttons[PS3_BUTTON_REAR_LEFT_1])
    {
        cmd.linear.x = walk_vel*joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
        cmd.linear.y = walk_vel*joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
        cmd.linear.z = vertical_vel*joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
        cmd.angular.z = yaw_rate*joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
    }
    else if(joy->buttons[PS3_BUTTON_REAR_RIGHT_1])
    {
        cmd.linear.x = run_vel*joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
        cmd.linear.y = run_vel*joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
        cmd.linear.z = vertical_vel*joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
        cmd.angular.z = yaw_rate_run*joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
    }
    //Gripper
    if(joy->buttons[PS3_AXIS_BUTTON_ACTION_CROSS])
    {
        gripper.data -= 0.01;
        gripper.data = gripper.data<0?0:gripper.data;
        dirtygripper = true;
    }
    else if(PS3_AXIS_BUTTON_ACTION_CIRCLE)
    {
        gripper.data += 0.01;
        gripper.data = gripper.data>0.5?0.5:gripper.data;
        dirtygripper = true;
    }
        vel_pub_.publish(cmd);
    if(dirtygripper==true)
        grip_pub_.publish(gripper);


}

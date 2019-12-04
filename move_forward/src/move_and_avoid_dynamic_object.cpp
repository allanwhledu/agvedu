#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <signal.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <std_msgs/Int32.h>
#include "move_forward/agv_call.h"

//先请求了movebase的服务
//然后又请求了一次自定义的服务

int switch_tar = 1;
int first_loop = 1;

void target_Callback(const std_msgs::Int32::ConstPtr& msg)
{
    switch_tar = msg->data;
    }


int main(int argc, char** argv)
{

    ros::init(argc, argv, "jrc_move2_initialpose");
    ros::NodeHandle node;

    // pub
    ros::Publisher initial_pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100);

    // sub
    ros::Subscriber scan_sub = node.subscribe<std_msgs::Int32>("/target",1,target_Callback);

    // server
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    // ac是一个client

    // 调用回调函数，获得初始的位姿和目标
    int initial_time_counter = 0;

    //read the parameter from config file
    double initial_pose_x, initial_pose_y, initial_pose_th;
    double target_pose_x, target_pose_y, target_pose_th;

    // 主循环
    ros::Rate loop_rate(5);
    while (ros::ok()){

        ros::spinOnce();

        ros::param::get("initial_pose_x", initial_pose_x);
        ros::param::get("initial_pose_y", initial_pose_y);
        ros::param::get("initial_pose_th", initial_pose_th);

        switch (switch_tar){
            case 1:
                ros::param::get("target_pose_x1", target_pose_x);
                ros::param::get("target_pose_y1", target_pose_y);
                ros::param::get("target_pose_th1", target_pose_th);
            case 2:
                ros::param::get("target_pose_x2", target_pose_x);
                ros::param::get("target_pose_y2", target_pose_y);
                ros::param::get("target_pose_th2", target_pose_th);
        }
        //todo 可以给出多个参数，然后订阅一下节点，依靠订阅消息来装入不同的参数吧

        geometry_msgs::Pose  target_pose;
        target_pose.position.x = target_pose_x;
        target_pose.position.y = target_pose_y;
        target_pose.position.z = 0;
        target_pose.orientation = tf::createQuaternionMsgFromYaw(target_pose_th);

        //publish the initial pose
        geometry_msgs::PoseWithCovarianceStamped initial_pose;//initial_pose_x
        initial_pose.header.stamp = ros::Time(0);
        initial_pose.header.frame_id = "/map";
        initial_pose.pose.pose.position.x = initial_pose_x;//-4.402, 1.547
        initial_pose.pose.pose.position.y = initial_pose_y;
        initial_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(initial_pose_th);


        initial_time_counter = 0;
        // 首先5次initial pose
        while(initial_time_counter < 5 && first_loop == 1)
        {
            ros::spinOnce();
            initial_pose_pub.publish(initial_pose);
            ROS_INFO("the robot initial pose");
            initial_time_counter = initial_time_counter + 1;
            sleep(1);
        }


        //move  to target position
        // 先连接一个move base是否可用
        ROS_INFO("move_base_square.cpp start...");
        //Wait 60 seconds for the action server to become available
        if(!ac.waitForServer(ros::Duration(60)))
        {
            ROS_INFO("Can't connected to move base server");
            return 1;
        }

        //Intialize the waypoint goal
        // 将目标点装入
        move_base_msgs::MoveBaseGoal goal;

        //Use the map frame to define goal poses
        goal.target_pose.header.frame_id = "/map";

        //Set the time stamp to "now"
        goal.target_pose.header.stamp = ros::Time::now();

        //Set the goal pose to the i-th waypoint
        goal.target_pose.pose = target_pose;

        //Start the robot moving toward the goal
        //Send the goal pose to the MoveBaseAction server
        // 这里应该是在请求服务了
        ac.sendGoal(goal);

        //Allow 1 minute to get there
        bool finished_within_time = ac.waitForResult(ros::Duration(180));

        //If we dont get there in time, abort the goal
        if(!finished_within_time)
        {
            ac.cancelGoal();
            ROS_INFO("Timed out achieving goal");
        }
        else
        {
            //We made it!
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Goal succeeded!");
            }
            else
            {
                ROS_INFO("The base failed for some reason");
            }
        }

        first_loop = 0;
        loop_rate.sleep();
    }

return 0;

}


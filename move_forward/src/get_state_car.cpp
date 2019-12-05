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

// 按键控制信号
int switch_tar = 0;

// 状态变量
bool first_loop = true;
bool call_movebase = true;
bool goal_changed = false;

void target_Callback(const std_msgs::Int32::ConstPtr& msg){
    if(switch_tar != msg->data)
        goal_changed = true;

    switch_tar = msg->data;
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "get_state_car");
    ros::NodeHandle node;

    // pub
    ros::Publisher agv_state_pub = node.advertise<std_msgs::Int32>("/agv_state", 100);

    // server
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);


    // 主循环
    ros::Rate loop_rate(5);
    while (ros::ok()){
        ros::spinOnce();
        std_msgs::Int32 agv_state_msg;
        agv_state_msg.data=0;


        bool finished_within_time = ac.waitForResult(ros::Duration(1));

        //If we dont get there in time, abort the goal
        if(!finished_within_time)
        {
            ac.cancelGoal();
            agv_state_msg.data=2;
            ROS_INFO("still getting to goal");
        }
        else
        {
            //We made it!
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                agv_state_msg.data=1;
                ROS_INFO("Goal succeeded!");
            }
            else
            {
                agv_state_msg.data=0;
                ROS_INFO("The base failed for some reason");
            }
        }
        agv_state_pub.publish(agv_state_msg);
        first_loop = false;
        loop_rate.sleep();
    }

    return 0;

}
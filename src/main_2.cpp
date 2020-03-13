/*
#include  <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include"../inc/cxz_robot_arm.h"
double new_joint_angle[6];
void cxz::SpeedCurve(const double t,double (&expected_speed)[6]);

void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  new_joint_angle[0]=msg->position[0];
  new_joint_angle[1]=msg->position[1];
  new_joint_angle[2]=msg->position[2];
  new_joint_angle[3]=msg->position[3];
  new_joint_angle[4]=msg->position[4];
  new_joint_angle[5]=msg->position[5];

}

int main(int argc,char *argv[])
{
    ros::init(argc, argv, "vel_control");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
    ros::Publisher vel_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 100);
    std_msgs::Float64MultiArray init_vel;
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    sleep(1);

    double t,target_speed[6],joint_speed[6];
    cxz::RobotArm robotarm;

    ros::Subscriber sub = node_handle.subscribe("/probot_anno/joint_states", 100, jointstatesCallback);
    ros::Rate loop_rate(20);
    ros::Time begin = ros::Time::now();    
    while (ros::ok()  && ((ros::Time::now()-begin).toSec()<12.1))
    {
        ros::spinOnce();
        t=(ros::Time::now()-begin).toSec();

        cxz::SpeedCurve(t,target_speed);
        robotarm.UpdateAngle(new_joint_angle);
        robotarm.SpeedControl(target_speed,joint_speed);
        for(size_t i=0;i<6;i++)
            init_vel.data.at(i) = joint_speed[i];            
        vel_pub.publish(init_vel);

        loop_rate.sleep();
    }

    return 0;
}
*/
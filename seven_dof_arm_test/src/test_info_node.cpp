// 包含miveit的API头文件
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <stdio.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_info_node");
  ros::NodeHandle node_handle;  
  tf::TransformListener listener;
  ros::Rate rate(10.0);

  // 设置机器人终端的目标位置
  // 设置机器人终端的目标位置
  geometry_msgs::PoseStamped  current_ee_link_pose;
  geometry_msgs::PoseStamped  current_base_link_pose;

  while(node_handle.ok())
  {
    moveit::planning_interface::MoveGroup group("manipulator");
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
    //
    current_ee_link_pose=group.getCurrentPose("ee_link");
    current_base_link_pose=group.getCurrentPose("base_link");

    ROS_INFO_STREAM("current_ee_link_pose"<<current_ee_link_pose);
    ROS_INFO_STREAM("current_base_link_pose"<<current_base_link_pose);


    //listener.transformPose("base_ee_link",current_ee_link_pose,current_base_link_pose);
    rate.sleep();

  }

  return 0;
};
// 包含miveit的API头文件
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <stdio.h>
#include <boost/thread.hpp>
// 设置机器人终端的目标位置
// 设置机器人终端的目标位置
static geometry_msgs::PoseStamped target_pose1;
geometry_msgs::Pose  target_pose_last;
geometry_msgs::PoseStamped  current_pose1;
moveit::planning_interface::MoveGroup::Plan my_plan;


void chatterCallback(const geometry_msgs::Pose& getPose)
{
  ROS_INFO_STREAM("chatterCallback: thread=" << boost::this_thread::get_id() << "]");

  //printf current_pose1;
  target_pose1.header.frame_id = "/world";
  target_pose1.header.stamp = ros::Time::now();

  target_pose1.pose.orientation.w = getPose.orientation.w;
  target_pose1.pose.orientation.x= getPose.orientation.x;
  target_pose1.pose.orientation.y = getPose.orientation.y;
  target_pose1.pose.orientation.z = getPose.orientation.z;
  target_pose1.pose.position.x = getPose.position.x;
  target_pose1.pose.position.y = getPose.position.y;
  target_pose1.pose.position.z = getPose.position.z;
  
  ROS_INFO_STREAM("target_pose_inner"<<target_pose1);
  //
  if (  target_pose_last.orientation.w != getPose.orientation.w ||
  target_pose_last.orientation.x != getPose.orientation.x ||
  target_pose_last.orientation.y != getPose.orientation.y ||
  target_pose_last.orientation.z != getPose.orientation.z ||
  target_pose_last.position.x != getPose.position.x ||
  target_pose_last.position.y != getPose.position.y ||
  target_pose_last.position.z != getPose.position.z)
  {
    target_pose_last = getPose;
    moveit::planning_interface::MoveGroup group("manipulator");
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    //ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
    current_pose1=group.getCurrentPose("ee_link");
    
    ROS_INFO_STREAM("current_pose1"<<current_pose1);
    ROS_INFO_STREAM("target_pose1"<<target_pose1);
    //the purpose is if same position,do not plan a new way. BUT now this does not work，因为机械臂一直在小幅度震动，pose信息时刻在发生变化
    //可以添加阈值进行控制
    double tol = current_pose1.pose.orientation.x-target_pose1.pose.orientation.x;
    ROS_INFO_STREAM("go before"<<tol);
    if(
        fabs(current_pose1.pose.orientation.w-target_pose1.pose.orientation.w)<0.01      
      &&fabs(current_pose1.pose.orientation.x-target_pose1.pose.orientation.x)<0.01
      &&fabs(current_pose1.pose.orientation.y-target_pose1.pose.orientation.y)<0.01
      &&fabs(current_pose1.pose.orientation.z-target_pose1.pose.orientation.z)<0.01
      &&fabs(current_pose1.pose.position.x-target_pose1.pose.position.x)<0.01
      &&fabs(current_pose1.pose.position.y-target_pose1.pose.position.y)<0.01
      &&fabs(current_pose1.pose.position.z-target_pose1.pose.position.z)<0.01
      )
    {
      ROS_INFO_STREAM("go after"<<tol);
    }else if(  target_pose1.pose.orientation.w == 0 &&
      target_pose1.pose.orientation.x==0 && 
      target_pose1.pose.orientation.y ==0 &&
      target_pose1.pose.orientation.z ==0 &&
      target_pose1.pose.position.x ==0 &&
      target_pose1.pose.position.y ==0&&
      target_pose1.pose.position.z ==0)
    {
      ROS_INFO_STREAM("no target pose");
    }
    else
    {
      group.setPlannerId("RRTConnectkConfigDefault");
      group.allowLooking(true);

      group.setGoalPositionTolerance(0.01);
      group.setGoalOrientationTolerance(0.01);
      group.setMaxVelocityScalingFactor(0.2);
      group.setPoseTarget(target_pose1);
      //group.setGoalTolerance(0.01);
      
      //ROS_INFO_STREAM("test1");
      // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
      
      //ROS_INFO_STREAM("test2");

      //bool success = group.plan(my_plan);
      //ROS_INFO_STREAM("test3");
      
      //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   
     
      //让机械臂按照规划的轨迹开始运动。
      //if(success)
      //    group.execute(my_plan);
      //group.setStartStateToCurrentState();
      int errorcode = group.move();
      ROS_INFO_STREAM("errorcode"<<errorcode);
      if(errorcode == 0){
          group.setPlannerId("BKPIECEkConfigDefault");
          group.move();
        }
      //current_pose1=group.getCurrentPose("ee_link");
      group.setStartStateToCurrentState();
      current_pose1=group.getCurrentPose("ee_link");

      ROS_INFO_STREAM("current_pose1testttttt"<<target_pose1);
      ROS_INFO_STREAM("current_pose1testttttt"<<current_pose1);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_listener",ros::init_options::AnonymousName);
  ros::NodeHandle node_handle_custom;  
  //init target_pose_last 
  target_pose_last.orientation.w = 0;
  target_pose_last.orientation.x= 0;
  target_pose_last.orientation.y = 0;
  target_pose_last.orientation.z = 0;
  target_pose_last.position.x = 0;
  target_pose_last.position.y = 0;
  target_pose_last.position.z = 0;

  target_pose1.header.frame_id = "/world";
  target_pose1.header.stamp = ros::Time::now();
  target_pose1.pose.orientation.w = 0;
  target_pose1.pose.orientation.x= 0;
  target_pose1.pose.orientation.y = 0;
  target_pose1.pose.orientation.z = 0;
  target_pose1.pose.position.x = 0;
  target_pose1.pose.position.y = 0;
  target_pose1.pose.position.z = 0;


  ros::AsyncSpinner spinner(1);
  spinner.start();

//ros::spin();
  //while(1)
  
    ROS_INFO_STREAM("Main thread [" << boost::this_thread::get_id() << "].");
    ros::Subscriber sub = node_handle_custom.subscribe("/geometry", 1, chatterCallback); 
  ros::spin();
  //ROS_INFO_STREAM_ONCE("This appears only once.");

  //ros::Subscriber sub = node_handle.subscribe("/geometry", 10, chatterCallback);


// END_TUTORIAL
  //
  //ros::shutdown();
  //ROS_INFO_STREAM("shutdown");
  ros::waitForShutdown();
  return 0;
}

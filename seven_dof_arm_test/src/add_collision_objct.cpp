// 包含API的头文件
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_collision_objct");
    ros::NodeHandle nh_add_coll;
    ros::AsyncSpinner spin(1);
    spin.start();
 
    // 创建运动规划的情景，等待创建完成
    moveit::planning_interface::PlanningSceneInterface current_scene;
    sleep(1.0);
 
    // 声明一个障碍物的实例，并且为其设置一个id，方便对其进行操作，该实例会发布到当前的情景实例中
    moveit_msgs::CollisionObject cylinder;
    cylinder.id = "seven_dof_arm_cylinder";
 
    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.56;
    primitive.dimensions[1] = 0.185;
 
    // 设置障碍物的位置
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    pose.position.x =  0.64000;
    pose.position.y =  0.000000;
    pose.position.z =  -0.280000;
 
    // 将障碍物的属性、位置加入到障碍物的实例中
    cylinder.primitives.push_back(primitive);
    cylinder.primitive_poses.push_back(pose);
    cylinder.operation = cylinder.ADD;


    // 声明一个障碍物的实例，并且为其设置一个id，方便对其进行操作，该实例会发布到当前的情景实例中
    moveit_msgs::CollisionObject boxUp;
    boxUp.id = "seven_dof_arm_boxUp";
 
    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive primitiveUp;
    primitiveUp.type = primitiveUp.BOX;
    primitiveUp.dimensions.resize(3);
    primitiveUp.dimensions[0] = 0.20;
    primitiveUp.dimensions[1] = 0.24;
    primitiveUp.dimensions[2] = 0.56;
 
    // 设置障碍物的位置
    geometry_msgs::Pose poseUp;
    poseUp.orientation.w = 1;
    poseUp.position.x =  0.64000;
    poseUp.position.y =  0.000000;
    poseUp.position.z =  -0.280000;
 
    // 将障碍物的属性、位置加入到障碍物的实例中
    boxUp.primitives.push_back(primitiveUp);
    boxUp.primitive_poses.push_back(poseUp);
    boxUp.operation = boxUp.ADD;


    // 声明一个障碍物的实例，并且为其设置一个id，方便对其进行操作，该实例会发布到当前的情景实例中
    moveit_msgs::CollisionObject object;
    object.id = "seven_dof_arm_object";
 
    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 0.11;
    primitive2.dimensions[1] = 0.16;
    primitive2.dimensions[2] = 0.19;
    //7,5.5,13
    // 设置障碍物的位置
    geometry_msgs::Pose pose2;
    pose2.orientation.w = 1;
    pose2.position.x =  0.640000;
    pose2.position.y =  0.000000;
    pose2.position.z =  0.0750000;
 
    // 将障碍物的属性、位置加入到障碍物的实例中
    object.primitives.push_back(primitive2);
    object.primitive_poses.push_back(pose2);
    object.operation = object.ADD;

    // 声明一个障碍物的实例，并且为其设置一个id，方便对其进行操作，该实例会发布到当前的情景实例中
    moveit_msgs::CollisionObject box_right;
    box_right.id = "box_right";
 
    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive primitiveRight;
    primitiveRight.type = primitiveRight.BOX;
    primitiveRight.dimensions.resize(3);
    primitiveRight.dimensions[0] = 2.00;
    primitiveRight.dimensions[1] = 0.16;
    primitiveRight.dimensions[2] = 4.00;
 
    // 设置障碍物的位置
    geometry_msgs::Pose poseRight;
    poseRight.orientation.w = 1;
    poseRight.position.x =  0.000000;
    poseRight.position.y =  -1.040000;
    poseRight.position.z =  -1.00000;
 
    // 将障碍物的属性、位置加入到障碍物的实例中
    box_right.primitives.push_back(primitiveRight);
    box_right.primitive_poses.push_back(poseRight);
    box_right.operation = box_right.ADD;

    // 声明一个障碍物的实例，并且为其设置一个id，方便对其进行操作，该实例会发布到当前的情景实例中
    moveit_msgs::CollisionObject box_left;
    box_left.id = "box_left";
 
    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive primitiveLeft;
    primitiveLeft.type = primitiveLeft.BOX;
    primitiveLeft.dimensions.resize(3);
    primitiveLeft.dimensions[0] = 0.8;
    primitiveLeft.dimensions[1] = 0.1;
    primitiveLeft.dimensions[2] = 0.74;
 
    // 设置障碍物的位置
    geometry_msgs::Pose poseLeft;
    poseLeft.orientation.w = 1;
    poseLeft.position.x =  -0.130000;
    poseLeft.position.y =  0.9850000;
    poseLeft.position.z =  -0.22000;
 
    // 将障碍物的属性、位置加入到障碍物的实例中
    box_left.primitives.push_back(primitiveLeft);
    box_left.primitive_poses.push_back(poseLeft);
    box_left.operation = box_left.ADD;



    // 声明一个障碍物的实例，并且为其设置一个id，方便对其进行操作，该实例会发布到当前的情景实例中
    moveit_msgs::CollisionObject box_bottom;
    box_bottom.id = "box_bottom";
 
    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive primitiveBottom;
    primitiveBottom.type = primitiveBottom.BOX;
    primitiveBottom.dimensions.resize(3);
    primitiveBottom.dimensions[0] = 2.0;
    primitiveBottom.dimensions[1] = 2.0;
    primitiveBottom.dimensions[2] = 0.1;
 
    // 设置障碍物的位置
    geometry_msgs::Pose poseBottom;
    poseBottom.orientation.w = 1;
    poseBottom.position.x =  0.00000;
    poseBottom.position.y =  0.00000;
    poseBottom.position.z =  -0.36000;
 
    // 将障碍物的属性、位置加入到障碍物的实例中
    box_bottom.primitives.push_back(primitiveBottom);
    box_bottom.primitive_poses.push_back(poseBottom);
    box_bottom.operation = box_bottom.ADD;




    // 声明一个障碍物的实例，并且为其设置一个id，方便对其进行操作，该实例会发布到当前的情景实例中
    moveit_msgs::CollisionObject box_bottom_cy;
    box_bottom_cy.id = "box_bottom_cy";
 
    // 设置障碍物的外形、尺寸等属性   
    shape_msgs::SolidPrimitive primitiveBottomCy;
    primitiveBottomCy.type = primitiveBottomCy.BOX;
    primitiveBottomCy.dimensions.resize(3);
    primitiveBottomCy.dimensions[0] = 0.16;
    primitiveBottomCy.dimensions[1] = 0.26;
    primitiveBottomCy.dimensions[2] = 0.56;
 
    // 设置障碍物的位置
    geometry_msgs::Pose poseBottomCy;
    poseBottomCy.orientation.w = 1;
    poseBottomCy.position.x =  0.00000;
    poseBottomCy.position.y =  0.00000;
    poseBottomCy.position.z =  -0.28000;
 
    // 将障碍物的属性、位置加入到障碍物的实例中
    box_bottom_cy.primitives.push_back(primitiveBottomCy);
    box_bottom_cy.primitive_poses.push_back(poseBottomCy);
    box_bottom_cy.operation = box_bottom_cy.ADD;


 
    // 创建一个障碍物的列表，把之前创建的障碍物实例加入其中
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    //collision_objects.push_back(cylinder);
    collision_objects.push_back(object);
    collision_objects.push_back(box_right);
    collision_objects.push_back(box_left);
    collision_objects.push_back(boxUp);
    //collision_objects.push_back(boxBoard);
    collision_objects.push_back(box_bottom);
    collision_objects.push_back(box_bottom_cy);
    // 所有障碍物加入列表后（这里只有一个障碍物），再把障碍物加入到当前的情景中，如果要删除障碍物，使用removeCollisionObjects(collision_objects)
    current_scene.addCollisionObjects(collision_objects);
 
    ros::shutdown();
 
    return 0;
}

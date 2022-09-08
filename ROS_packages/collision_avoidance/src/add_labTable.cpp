/**
 * @file add_labTable.cpp
 * @author Bonafini Marco
 * @brief ROS node that adds the laboratory's table as collision object 
 * @copyright Copyright (c) 2022
 *
 */
#include<iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ros/ros.h"
#include <cstdlib>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <signal.h>
#include <string>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

///Define moveit group name [Arm]
const std::string PLANNING_GROUP_ARM = "manipulator"; 

///Pointer to the planning scene
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
///pointer to the arm move group
moveit::planning_interface::MoveGroupInterface *move_group_arm;


///Function to add collision objects
///
///The function extract the mesh of the LabTable and adds it to the scene
void add_collision_obj(){
    
    
    //VECTOR FOR COLLISION OBJECTS
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    Eigen::Vector3d scale(1.02, 1.02, 1.02);
    collision_objects[0].id = "table";
    shapes::Mesh* m = shapes::createMeshFromResource("package://collision_avoidance/meshes/labTable_meshes/table_rob_lab.dae", scale); 
    ROS_INFO("table mesh loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;  
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    // ADDING LAB TABLE AS COLLISION OBJECT 
    // ----------------------------------------------------------------------------------------------------------------------------------------------------

    collision_objects[0].meshes.resize(1);
    collision_objects[0].mesh_poses.resize(1);
    collision_objects[0].meshes[0] = mesh;
    collision_objects[0].header.frame_id = move_group_arm->getPlanningFrame();   
    collision_objects[0].mesh_poses[0].position.x = 0.5;
    collision_objects[0].mesh_poses[0].position.y = -0.35;
    collision_objects[0].mesh_poses[0].position.z = 1.82;
    collision_objects[0].mesh_poses[0].orientation.x= 0.0; 
    collision_objects[0].mesh_poses[0].orientation.y= 3.14;
    collision_objects[0].mesh_poses[0].orientation.z= 0.0;   

    collision_objects[0].meshes.push_back(mesh);
    collision_objects[0].mesh_poses.push_back(collision_objects[0].mesh_poses[0]);
    collision_objects[0].operation = collision_objects[0].ADD;
    
    // ----------------------------------------------------------------------------------------------------------------------------------------------------
    // WALL BEHIND TABLE

    collision_objects[1].header.frame_id = move_group_arm->getPlanningFrame();
    collision_objects[1].id = "wall";

    //shape_msgs::SolidPrimitive primitives;
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 1;
    collision_objects[1].primitives[0].dimensions[1] = 0.1;
    collision_objects[1].primitives[0].dimensions[2] = 1;

    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.0;
    collision_objects[1].primitive_poses[0].position.y = -0.45;
    collision_objects[1].primitive_poses[0].position.z = 0.43;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    
    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface->applyCollisionObjects(collision_objects);
    ROS_INFO("objects added to the world");
}

///Function to Remove the collision objects
///
///When the node is terminated the collision objects are removed from the scene
void remove_collision_obj(int sig){
    std::vector<std::string> object_ids;
    object_ids.push_back("table");
    object_ids.push_back("wall");
    planning_scene_interface->removeCollisionObjects(object_ids);
    ROS_INFO("objects removed from the world");
    ros::shutdown();
}

///Main Function
///
///It initializes the ROS node, moves the robot to rest pose and adds the Laboratory table as collision object
int main(int argc,char** args){
    ros::init(argc, args, "add_objects", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, remove_collision_obj);

    move_group_arm = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface;

    add_collision_obj();
    sleep(1);
    move_group_arm->setJointValueTarget(move_group_arm->getNamedTargetValues("home1"));
    move_group_arm->move();
 
    while (ros::ok()){
        ros::Duration(0.5).sleep();
    }
}

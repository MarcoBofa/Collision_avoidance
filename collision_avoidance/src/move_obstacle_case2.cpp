/**
 * @file move_obstacle_case2.cpp
 * @author Bonafini Marco
 * @brief ROS node that moves case2 obstacles when signaled by algoritmo_prevenzione_collisioni.cpp
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
#include "gazebo_msgs/ModelState.h"
#include <gazebo_msgs/SetModelState.h>
#include "std_msgs/String.h"
#include <thread>

///Define moveit group name [Arm]
const std::string PLANNING_GROUP_ARM = "manipulator"; 

///Pointer to the planning scene
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
///pointer to the arm move group
moveit::planning_interface::MoveGroupInterface *move_group_arm;

///Booloean value that control the object motion
bool move = true;

///Function to Remove the collision objects
void remove_collision_objects(std::string id){
  std::cout << "\033[32m" << "Objects removed from the world " << "\033[0m" << std::endl;
  std::vector<std::string> objects_ids;
  objects_ids.push_back(id);
  planning_scene_interface->removeCollisionObjects(objects_ids);
  usleep(1000);
  std::cout << "------------------------------------------------------------------------------------------------------------------" << std::endl << std::endl;
  
}

///Function that moves the gazebo obstacle
///
///The gazebo model of the obstacle is moved by publishing a new pose
///@param node_handle Main mechanism to interact with the ROS system. It's used to register the program as a node with the ROS master
///@param obj_pos vector containing the new object position and orientation
void publish_model_pose(ros::NodeHandle n, std::vector<double>& obj_pos) {

  gazebo_msgs::ModelState model_msg;
  model_msg.model_name = "o_obs";
  model_msg.pose.position.x = -obj_pos[0];
  model_msg.pose.position.y = obj_pos[1];
  model_msg.pose.position.z = obj_pos[2]+0.15;

  ros::Publisher pub; 
  pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
  while (pub.getNumSubscribers() < 1) {
    // wait for a connection to publisher
  }

  pub.publish(model_msg);

}

///Function that moves the collision object
///
///@param obj_pos vector containing the new object position and orientation
///@param id ID of the collision object to move
void move_collision_objects(ros::NodeHandle n, std::vector<double>& obj_pos, std::string id){

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm->getPlanningFrame();
  collision_object.id = id;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = obj_pos[3];
  primitive.dimensions[primitive.BOX_Y] = obj_pos[4];
  primitive.dimensions[primitive.BOX_Z] = obj_pos[5];

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = obj_pos[0];
  box_pose.position.y = obj_pos[1];
  box_pose.position.z = obj_pos[2];
  box_pose.orientation.w = obj_pos[6];
  box_pose.orientation.x = obj_pos[7];
  box_pose.orientation.z = obj_pos[8];

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface->addCollisionObjects(collision_objects);

  usleep(1000);

}

///Function that adds collision objects
///
///The function adds all the collision objects representing a human
void add_collision_obj(){

    remove_collision_objects("o_obs");
    
  // ADD ORIZONTAL OBSTACLE   -------------------------------------------------------------

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm->getPlanningFrame();
  collision_object.id = "o_obs";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 0.9;
  primitive.dimensions[primitive.BOX_Z] = 0.1;

  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0.06;
  box_pose.position.y = 0.4;
  box_pose.position.z = 0.79;
  box_pose.orientation.w = 1;
  box_pose.orientation.x = 0.06;
  box_pose.orientation.z = 0.01;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

    planning_scene_interface->applyCollisionObjects(collision_objects);
    ROS_INFO_NAMED("add_objects", "objects added to the world");
}

///Function that wait for the message to stop the collision object motion
///
///The function subscribe to the topic where the collision avoidance node will publish the message to stop the 
///collision object motion
void wait_for_stop() {

  std_msgs::String::ConstPtr msg = ros::topic::waitForMessage<std_msgs::String>("/stop_move");
  std::cout<<"STOP obstacle movement" << std::endl;
  remove_collision_objects("o_obs");
  move = false;
  sleep(1);
  ros::shutdown();
}

///Function that wait for the message to start the collision object motion
///
///The function subscribe to the topic where the collision avoidance node will publish the message to start the 
///collision object motion
void wait_for_message(ros::NodeHandle n) {

    std_msgs::String::ConstPtr msg = ros::topic::waitForMessage<std_msgs::String>("/chatter");
    std::cout<<"START obstacle movement" << std::endl;
    std::vector<double> obj_pos(9);
    
    obj_pos[0] = 0.06; obj_pos[1] = 0.4; obj_pos[2] = 0.79; obj_pos[3] = 0.1; obj_pos[4] = 0.9; obj_pos[5] = 0.1; 
    obj_pos[6] = 1.0;  obj_pos[7] = 0.06; obj_pos[8] = 0.01;

    //start the thread that will wait for the message to stop the motion of the obstacle
    std::thread thread_obj(wait_for_stop);
    move = true;
    while(move) {
        if(obj_pos[2] <= 0.51) {
           break;
        }
        
        obj_pos[7] -= 0.00002;
        obj_pos[2] -= 0.00013;
        move_collision_objects(n, obj_pos, "o_obs");
        //obj_pos[7] += 0.0045; if gazebo obstacle motion
        //publish_model_pose(n, obj_pos);
        usleep(10000);
    }
    thread_obj.join();

    move = true;
}

///Main Function
///
///It initializes the ROS node, moves the robot to rest pose and starts the subscriber for the obstacle motion
int main(int argc,char** args){

    ros::init(argc, args, "move_obstacle2_node");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);

    move_group_arm = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface;

    // std::vector<double> obj_pos(3);
    // obj_pos[0] = -0.14; obj_pos[1] = 0.25; obj_pos[2] = 0.6;
    // publish_model_pose(n, obj_pos);

    add_collision_obj();

    spinner.start();

    sleep(2);
    wait_for_message(n);
    std::cout << "exited " << std::endl;

    //ros::spin();
    return 0;
}
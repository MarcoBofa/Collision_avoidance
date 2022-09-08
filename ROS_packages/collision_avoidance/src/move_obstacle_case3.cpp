/**
 * @file move_obstacle_case3.cpp
 * @author Bonafini Marco
* @brief ROS node that moves case3 obstacles when signaled by algoritmo_prevenzione_collisioni.cpp
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
    model_msg.model_name = "arm_right";
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
void move_collision_objects(std::vector<double>& obj_pos, std::string id){

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
void addCollisions(){
    
    
    //VECTOR FOR COLLISION OBJECTS
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(5);

    remove_collision_objects("body");
    remove_collision_objects("arm_left");
    remove_collision_objects("arm_right");
    remove_collision_objects("head");
    remove_collision_objects("seleing");

    

    // RIGHT ARM   -------------------------------------------------------------

    collision_objects[1].header.frame_id = move_group_arm->getPlanningFrame();
    collision_objects[1].id = "arm_right";
    
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.1;
    collision_objects[1].primitives[0].dimensions[1] = 0.7;
    collision_objects[1].primitives[0].dimensions[2] = 0.1;

    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.13;
    collision_objects[1].primitive_poses[0].position.y = 0.38;
    collision_objects[1].primitive_poses[0].position.z = 0.79;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects[1].primitive_poses[0].orientation.x = -0.005;
    collision_objects[1].primitive_poses[0].orientation.z = -0.05;

    collision_objects[1].operation = collision_objects[1].ADD;


    // LEFT ARM   -------------------------------------------------------------

    collision_objects[4].header.frame_id = move_group_arm->getPlanningFrame();
    collision_objects[4].id = "arm_left";
  
    collision_objects[4].primitives.resize(1);
    collision_objects[4].primitives[0].type = collision_objects[4].primitives[0].BOX;
    collision_objects[4].primitives[0].dimensions.resize(3);
    collision_objects[4].primitives[0].dimensions[0] = 0.1;
    collision_objects[4].primitives[0].dimensions[1] = 0.7;
    collision_objects[4].primitives[0].dimensions[2] = 0.1;

    collision_objects[4].primitive_poses.resize(1);
    collision_objects[4].primitive_poses[0].position.x = -0.21;
    collision_objects[4].primitive_poses[0].position.y = 0.42;
    collision_objects[4].primitive_poses[0].position.z = 0.87;
    collision_objects[4].primitive_poses[0].orientation.w = 1.0;

    collision_objects[4].operation = collision_objects[4].ADD;



    // BODY  -------------------------------------------------------------

    collision_objects[0].header.frame_id = move_group_arm->getPlanningFrame();
    collision_objects[0].id = "body";
    
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.35;
    collision_objects[0].primitives[0].dimensions[1] = 0.35;
    collision_objects[0].primitives[0].dimensions[2] = 1.5;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.9;
    collision_objects[0].primitive_poses[0].position.z = 1.1;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[0].operation = collision_objects[0].ADD;


    // HEAD  -------------------------------------------------------------

    collision_objects[3].header.frame_id = move_group_arm->getPlanningFrame();
    collision_objects[3].id = "head";
    
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 0.24;
    collision_objects[3].primitives[0].dimensions[1] = 0.28;
    collision_objects[3].primitives[0].dimensions[2] = 0.33;

    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0.0;
    collision_objects[3].primitive_poses[0].position.y = 0.9;
    collision_objects[3].primitive_poses[0].position.z = 0.2;
    collision_objects[3].primitive_poses[0].orientation.w = 1.0;

    collision_objects[3].operation = collision_objects[3].ADD;

    // SEALING  -------------------------------------------------------------

    collision_objects[2].header.frame_id = move_group_arm->getPlanningFrame();
    collision_objects[2].id = "sealing";

    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 1.5;
    collision_objects[2].primitives[0].dimensions[1] = 1.5;
    collision_objects[2].primitives[0].dimensions[2] = 0.1;

    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.0;
    collision_objects[2].primitive_poses[0].position.y = 0.0;
    collision_objects[2].primitive_poses[0].position.z = -0.2;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;

    collision_objects[2].operation = collision_objects[2].ADD;


    planning_scene_interface->applyCollisionObjects(collision_objects);
    ROS_INFO_NAMED("add_objects", "objects added to the world");
}

///Function that wait for the message to stop the collision object motion
///
///The function subscribe to the topic where the collision avoidance node will publish the message to stop the 
///collision object motion
void wait_for_stop() {

    std_msgs::String::ConstPtr msg = ros::topic::waitForMessage<std_msgs::String>("/stop_move");
    std::cout<<"STOP moving object" << std::endl;
    move = false;

}

///Function that wait for the message to start the collision object motion
///
///The function subscribes to the topic where the collision avoidance node will publish the message to start the 
///collision object motion
void wait_for_message(){

    std_msgs::String::ConstPtr msg = ros::topic::waitForMessage<std_msgs::String>("/chatter");

    bool up = true;

    if (msg->data.c_str()[0] == 'u' ) {
        up = true;
    } else {
        up = false;
    }

    std::cout<<"START moving object" << std::endl;
    std::vector<double> obj_pos(9);

    double delta_height = 0.0;
    double delta_orientation = 0.0;

    if (up) {
        delta_orientation = 0.00001;
        delta_height = -0.00011;
        obj_pos[0] = 0.13; obj_pos[1] = 0.38; obj_pos[2] = 0.79; obj_pos[3] = 0.1; obj_pos[4] = 0.7; obj_pos[5] = 0.1; 
        obj_pos[6] = 1.0;  obj_pos[7] = -0.005; obj_pos[8] = -0.05;
    } else {
        delta_orientation = -0.00001;
        delta_height = 0.00011;
        obj_pos[0] = 0.13; obj_pos[1] = 0.38; obj_pos[2] = 0.6; obj_pos[3] = 0.1; obj_pos[4] = 0.7; obj_pos[5] = 0.1; 
        obj_pos[6] = 1.0;  obj_pos[7] = -0.01; obj_pos[8] = 0.05;
    }

    //start the thread that will wait for the message to stop the motion of the obstacle
    std::thread thread_obj(wait_for_stop);
    move = true;
    while(move) {
        if (up) {
            if(obj_pos[2] <= 0.58)
                break;
        } else {
            if(obj_pos[2] >= 0.82)
                break;
        }
        
        obj_pos[7] += delta_orientation;
        obj_pos[2] += delta_height;
        move_collision_objects(obj_pos, "arm_right");
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

    ros::init(argc, args, "move_obstacle_node");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);

    move_group_arm = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface;

    // std::vector<double> obj_pos(3);
    // obj_pos[0] = -0.14; obj_pos[1] = 0.25; obj_pos[2] = 0.6;
    // publish_model_pose(n, obj_pos);

    addCollisions();

    spinner.start();

    sleep(2);
    wait_for_message();
    sleep(3);
    wait_for_message();
    std::cout << "exited " << std::endl;

    //ros::spin();
    return 0;
}
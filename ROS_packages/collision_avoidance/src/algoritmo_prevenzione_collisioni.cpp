/**
 * @file algoritmo_prevenzione_collisioni.cpp
 * @author Bonafini Marco
 * @brief The algoirthm moves the robot from a starting position to a goal position, the trajectory executed during the
 * motion is constantly modified in order to avoid collision with the changing environment
 * @copyright Copyright (c) 2022
 *
 */

#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ros/ros.h"
#include <cstdlib>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <pluginlib/class_loader.h>
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include <gazebo_msgs/SetModelState.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <memory>
#include <string>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <unistd.h>
#include <chrono>

const double home_pos_x = 0.5;
const double home_pos_y = 0.3;
const double home_pos_z = 0.87;

const double orientation_w = 0.0001;
const double orientation_x = 0.0;
const double orientation_y = 0.0;
const double orientation_z = -1;

const double start_planning_time = 0.15;
const double robot_accellerations = 0.18;
///Goal pose tolerance
const double tolerance = 0.05;

int test_num;

///Define moveit group name [Arm]
const std::string PLANNING_GROUP_ARM = "manipulator";
///Define moveit group name [End-Effector]
const std::string PLANNING_GROUP_GRIPPER = "endeffector";

///Name of the planner that will be used
const std::string PLANNER = "RRTstar";
//const std::string PLANNER = "LazyPRMstar";


///pointer to the arm move group
moveit::planning_interface::MoveGroupInterface *arm_move_group;

///pointer to the gripper move group
moveit::planning_interface::MoveGroupInterface *gripper_move_group;

///Planning Scene monitor that will create the planning scene
planning_scene_monitor::PlanningSceneMonitorPtr  psm;

/// RobotModelLoader: http://docs.ros.org/noetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
robot_model_loader::RobotModelLoader *robot_model_load;

///Function to publish messages to the specified topic
///
///@param node_handle Main mechanism to interact with the ROS system. It's used to register the program as a node with the ROS master
///@param topic Name of the topic where the message will be published
///@param num Number of the simulation test selected
void publish(ros::NodeHandle n, std::string topic, int num) {

  std_msgs::String msg;
  std::stringstream ss;
  if (num == 2 ) {
    ss << "up";
  } else {
    ss << "down";
  }

  msg.data = ss.str();

  ros::Rate loop_rate(10);

  ros::Publisher pub;
  pub = n.advertise<std_msgs::String>(topic, 1000);
  while (pub.getNumSubscribers() < 1) {
    // wait for a connection to publisher
  }

  // publish the message 5 times to make sure it will be readed, 
  // the subscriber will read only one message and the other will be discarded
  for (int i = 0; i<5; i++) {
    pub.publish(msg);
    loop_rate.sleep();
  }
  std::cout << "MOVEMENT MESSAGE PUBLISHED" << std::endl;

}


///Function to print the name of the joints with the corrisponing angles
///
///@param joint_model_group Is used to keep track of the current robot pose and planning group.
///@param robot_state The state of the robot for which the joints values will be printed
void print_links_value(const moveit::core::JointModelGroup* joint_model_group, moveit::core::RobotState& robot_state){

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  std::vector<double> joint_values;
  robot_state.copyJointGroupPositions(joint_model_group, joint_values);
  std::cout << "FINAL STATE VALUES" << std::endl;
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
  std::cout << "joint " << joint_names[i].c_str() << "   :   " << joint_values[i] << std::endl;
  }
  std::cout << "---------------------------------------------------------------------------------------------------------------------" <<  std::endl <<  std::endl;

}

///Function to instantiate all the classes
///
///The move group for the arm, the Robot Model Loader and the Planning Scene Monitor
void intantiate_classes(){

  //instantiate the move group for the arm
  arm_move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);
  arm_move_group->setPlannerId(PLANNER);
  arm_move_group->setPlanningTime(4);
  arm_move_group->setMaxAccelerationScalingFactor(robot_accellerations);
  arm_move_group->setMaxVelocityScalingFactor(robot_accellerations);
  arm_move_group->setGoalTolerance(tolerance);

  robot_model_load = new robot_model_loader::RobotModelLoader("robot_description");

  //instantiate the planning scene monitor and start all the monitor to update the scene
  psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();
  psm->requestPlanningSceneState("/get_planning_scene");


}

///Function to move the robot to the starting position
///
///@param node_handle Main mechanism to interact with the ROS system. It's used to register the program as a node with the ROS master
void move_to_start(ros::NodeHandle& node_handle){

  const moveit::core::RobotModelPtr& robot_model = robot_model_load->getModel();
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP_ARM);

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "home");

  planning_scene_monitor::LockedPlanningSceneRW lps(psm);

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_link";
  goal_pose.pose.position.x = home_pos_x;
  goal_pose.pose.position.y = home_pos_y;
  goal_pose.pose.position.z = home_pos_z;
  goal_pose.pose.orientation.w = orientation_w;
  goal_pose.pose.orientation.x = orientation_x;
  goal_pose.pose.orientation.y = orientation_y;
  goal_pose.pose.orientation.z = orientation_z;

  //print_links_value(joint_model_group, lps->getCurrentStateNonConst());

  arm_move_group->setPoseTarget(goal_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = false;

   while(success == false ) {
    success = (arm_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success == false) usleep(10000);
    }

    std::cout << std::endl;
    std::cout << "Moving to starting position" << std::endl;
    arm_move_group->execute(my_plan);
    sleep(2);

    success = false;

}


///Function used to select the number of configuration
///
///To select the appropriate number of configuration the function uses the number of waypoints
///of the trajectory calculated by the planners
///@param waypoint_num The total number of waypoints (states) the trajectory consists of.
///@return The function returns a vector with two number: the first is the number of configuration selected for
///the first partial trajectory, the second is the number of configuration to execute at the next iteration step if the
///trajectory is still collision free.
std::vector<int> select_num_configurations(int waypoint_num) {

   std::vector<int> n;
   n.resize(2);

   if (waypoint_num >= 10 && waypoint_num <= 100 ) {
      n[0] = waypoint_num*0.1;
      n[1] = (waypoint_num*0.1)*2;

    } else if (waypoint_num >= 2) {
      n[0] = waypoint_num-1;
      n[1] = (waypoint_num-1)+2;

    } else if (waypoint_num > 100) {
      n[0] = 10;
      n[1] = 20;

    } else {
      n[0] = 1;
      n[1] = 2;
    }
     if (n[0] <= 0 ) n[0] = 1;
    if (n[1] <= 0 ) n[1] = 1;

  return n;

}

///Function for dynamic collision avoidance 
///
///The function performs an iterations loop for all the duration of the motion, at each step it calculates a 
///new trajectory (if the previous is in collision) to avoid obstacles
///@param node_handle Main mechanism to interact with the ROS system. It's used to register the program as a node with the ROS master
///@param pos Vector containing the goal position for the end effector, specified in terms of x,y,z
///@param motion Boolean value to distinguish bewtween different simulation tests
void plan_and_move(ros::NodeHandle& node_handle, std::vector<double>& pos, bool test){

  const moveit::core::RobotModelPtr& robot_model = robot_model_load->getModel();
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP_ARM);

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "home");

  planning_scene_monitor::LockedPlanningSceneRW lps(psm);
  moveit::core::RobotState& init_state = lps->getCurrentStateNonConst();
  moveit::core::RobotState& final_state = lps->getCurrentStateNonConst();

  arm_move_group->setPlanningTime(0.5);

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_link";
  goal_pose.pose.position.x = pos[0];
  goal_pose.pose.position.y = pos[1];
  goal_pose.pose.position.z = pos[2];
  goal_pose.pose.orientation.w = orientation_w;
  goal_pose.pose.orientation.x = orientation_x;
  goal_pose.pose.orientation.y = orientation_y;
  goal_pose.pose.orientation.z = orientation_z;

  //0.685204  -0.694183  -0.155444  0.156344

  arm_move_group->setPoseTarget(goal_pose);
  arm_move_group->setStartState(init_state);
  bool success;
  bool repeat = false;
  double cycle = start_planning_time;
  std::vector<int> config_num; config_num.resize(2);

  moveit_msgs::RobotState final_state_msgs;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveGroupInterface::Plan final_plan;
  moveit::planning_interface::MoveGroupInterface::Plan next_plan;

  robot_trajectory::RobotTrajectory traj(robot_model, joint_model_group);
  robot_trajectory::RobotTrajectory temp_traj = traj;
  robot_trajectory::RobotTrajectory next_traj = traj;
  traj.clear(); temp_traj.clear(); next_traj.clear();

  if (test) {
    publish(node_handle, "chatter", 2);
  } else {
    publish(node_handle, "chatter", 3);
  }

  auto start = std::chrono::steady_clock::now();
  usleep(500000);


  for (int i = 0; i<100; i++ ) {

    
    // reset planning time to default value at the beginning of each cycle
    cycle = start_planning_time;
    success = false;

    if(i>0) {
      usleep(1000);
      arm_move_group->setStartState(final_state);
      arm_move_group->setNumPlanningAttempts(1);
    }

    // First planning
    if(i==0 ) {

      // Try planning a new trajectory, if the planning fails a new attempt will be made 
      // until one is found or termination condition is satisfied;
      while(success == false ) {
        arm_move_group->setPlanningTime(cycle);
        if (cycle >= 2.55 ) {
          std::cout <<"CONDIZIONE DI TERMINAZIONE RAGGIUNTA" << std::endl;
          
        }
        success = (arm_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // after each failed attempt the planning time will be increased by 0.025
        if (success == false) {
          usleep(1000);
          cycle += 0.025;
        }
      }

      temp_traj.setRobotTrajectoryMsg(init_state, my_plan.trajectory_);

      auto num = temp_traj.getWayPointCount();
      config_num = select_num_configurations(num);

      if (config_num[0] > 4 )
          config_num[0]--;
      if (config_num[1] > 8 )
          config_num[1]--;

      //copy the first configuration to excecute
      traj.append(temp_traj, 0.1, 0, config_num[0] );
      next_traj.append(temp_traj, 0.1, config_num[0], config_num[1] );

      traj.getRobotTrajectoryMsg(final_plan.trajectory_);
      final_plan.planning_time_ = my_plan.planning_time_;

      arm_move_group->execute(final_plan);

      temp_traj.clear();
      final_state = traj.getLastWayPoint();
      traj.clear();

      repeat = false;

    } else {

      moveit::core::robotStateToRobotStateMsg(final_state, final_state_msgs);
      next_traj.getRobotTrajectoryMsg(next_plan.trajectory_);

      // if the trajectory calculated at the previous step is still valid it will be excecuted without planning a new one
      // the repeat boolean is to ensure that the check on the trajectory is done only when a new one is calculated
      if (planning_scene->isPathValid(final_state_msgs, next_plan.trajectory_) == false || repeat == true ) {

        next_traj.clear();
        while(success == false ) {
          arm_move_group->setPlanningTime(cycle);
          if (cycle >= 2.55 ) {
            std::cout <<"CONDIZIONE DI TERMINAZIONE RAGGIUNTA" << std::endl;
            publish(node_handle, "stop_move",4);
            ros::shutdown();
          }
          success = (arm_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success == false) {
              usleep(1000);
              cycle += 0.025;
            }
          }

          if(i==0) {
            temp_traj.setRobotTrajectoryMsg(init_state, my_plan.trajectory_);
          } else {
          temp_traj.setRobotTrajectoryMsg(final_state, my_plan.trajectory_);
          }

          auto num = temp_traj.getWayPointCount();
          config_num = select_num_configurations(num);


          //copy the first configuration to excecute
          traj.append(temp_traj, 0.1, 0, config_num[0] );
          next_traj.append(temp_traj, 0.1, config_num[0], config_num[1] );

          traj.getRobotTrajectoryMsg(final_plan.trajectory_);
          final_plan.planning_time_ = my_plan.planning_time_;

          arm_move_group->execute(final_plan);

          temp_traj.clear();
          traj.clear();

          repeat = false;


      } else {
        repeat = true;

        //sleep(1);
        size_t num = next_traj.getWayPointCount();
        next_traj.getRobotTrajectoryMsg(final_plan.trajectory_);
          final_plan.planning_time_ = my_plan.planning_time_;

          if (num != 0 ){
            arm_move_group->execute(final_plan);
          }
          temp_traj.clear();

          final_state = next_traj.getLastWayPoint();
          traj.clear();

      }
    }


    //Check End Effector Position to stop when Goal is Reached
    usleep(10000);
    geometry_msgs::PoseStamped gripper_pose = arm_move_group->getCurrentPose();

    if( abs(gripper_pose.pose.position.x - goal_pose.pose.position.x) < 0.3 &&
        abs(gripper_pose.pose.position.y - goal_pose.pose.position.y) < 0.25 &&
        abs(gripper_pose.pose.position.z - goal_pose.pose.position.z) < 0.5    ) 
        {
          arm_move_group->setStartState(final_state);
          arm_move_group->setGoalTolerance(0.01);
          arm_move_group->move();

          auto end = std::chrono::steady_clock::now();
          std::cout << "Elapsed time in seconds: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " sec" << std::endl;
          publish(node_handle, "stop_move",4);
          if (test_num == 3 ) {
            break;
          } else {
            exit(0);
          }
    }
  }


}


///Main Function
///
///It initializes the ROS node and asks the user for the test to execute
int main(int argc, char **args)
{
    ros::init(argc, args, "move_group_interface");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    std::vector<double> goal_pose;

    intantiate_classes();
    bool valid = false;


    while(!valid) {

      std::cout << "Inserire il test selezionato : [1] - [2] - [3]" << std::endl;
      std::cin >> test_num;

      if(test_num > 3 || test_num < 1 ) {
        std::cout << "Numero inserito non valido" << std::endl;
      } else {
        valid = true;
      }

    }

    goal_pose.resize(3);
    goal_pose[0] = -0.49;
    goal_pose[1] = 0.32;
    goal_pose[2] = 0.86;

    spinner.start();

    //move to starting position
    move_to_start(n);
    sleep(2);


    //start movement
    plan_and_move(n, goal_pose, true);
    sleep(1);

    if (test_num == 3) {

      goal_pose[0] = 0.45;
      goal_pose[1] = 0.32;
      goal_pose[2] = 0.86;
      plan_and_move(n, goal_pose, false);
    }

    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "std_msgs/msg/string.hpp"

#include "Motionplanning_arms.hpp"
#include "RobotTaskStatus.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <play_motion2_msgs/action/play_motion2.hpp>
#include <control_msgs/action/point_head.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <yaml-cpp/yaml.h>
#include <memory>
#include <chrono>
#include <vector>

// #include "pipeline.h"

using namespace std::chrono_literals;

// void CMD_v_closeGripper(PPLN_Parameters const * const p_parameters);

// void CMD_v_dropBrick(PPLN_Parameters const * const p_parameters);

// void CMD_v_goOverDropPoint(PPLN_Parameters const * const p_parameters);

// void CMD_v_goToReceive(PPLN_Parameters const * const p_parameters);

// void CMD_v_openGripper(PPLN_Parameters const * const p_parameters);

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create a shared pointer to your node
  auto node = std::make_shared<icr_Motionplanning_arms>();

  node->GripperControl("CLOSE");
  // You can spin in a separate thread or just call your function directly
  try {
    double lift_value = 0.0;   // Example lift value within limits
    // node->TorsoControl(lift_value);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in TorsoControl: %s ", e.what());
  }
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
  planning_scene_publisher_ = node->create_publisher<moveit_msgs::msg::PlanningScene>(
    "/planning_scene", rclcpp::QoS(1));

// Add obj table in pose 0.4 0.0 0.5
  geometry_msgs::msg::PoseStamped obstacle_pose;
  obstacle_pose.header.frame_id = "base_footprint";
  obstacle_pose.pose.position.x = 1.025600;
  obstacle_pose.pose.position.y = 0.109529;
  obstacle_pose.pose.position.z = 0.5;
  obstacle_pose.pose.orientation.x = 0.0;
  obstacle_pose.pose.orientation.y = 0.0;
  obstacle_pose.pose.orientation.z = 0.0;
  obstacle_pose.pose.orientation.w = 1.0;
  moveit_msgs::msg::PlanningScene planning_scene_msg =
    node->Add_Obstacle(obstacle_pose, "Table");


  enum State {
    MOVE_TO_PICKUP = 0,
    PICK_BLOCK = 1,
    MOVE_TO_PLACE = 2,
    PLACE_BLOCK = 3,
    COMPLETE = 4,
};
  //Comment if it creates a problem  
  planning_scene_publisher_->publish(planning_scene_msg);

    // Initialize variables for the Jenga task
    float table_height = 0.60; // Height of the table
    int current_layer = 0;
    int current_block_in_layer = 0; //can be 0 or 1
    int max_layers = 8;
    int blocks_per_layer = 2;
    double x_target = 0.64829;
    double y_target = -0.050535;
    double z_target;
    double block_height = 0.034;
    double block_spacing = 0.15; //space between blocks in the same layer
    double gripper_height = 0.20;
    
    State current_state = MOVE_TO_PICKUP;
    bool running = true;

    while (rclcpp::ok() && running) {
      RCLCPP_INFO(node->get_logger(), "Layer: %d/%d Block: %d/%d", 
                          current_layer , max_layers, 
                          current_block_in_layer, blocks_per_layer);
      if (current_state == MOVE_TO_PICKUP) { 
        geometry_msgs::msg::Pose pick_up_pose;
          pick_up_pose.position.x = 0.45829;
          pick_up_pose.position.y = -0.050535;
          pick_up_pose.position.z = 0.93268; 
          tf2::Quaternion original_quat;
          original_quat.setRPY(0.0, 1.57, 0.0); // Roll, Pitch, Yaw in radians
          pick_up_pose.orientation.x = original_quat.x();
          pick_up_pose.orientation.y = original_quat.y();
          pick_up_pose.orientation.z = original_quat.z();
          pick_up_pose.orientation.w = original_quat.w();
         try {
                node->motion_planning_control(pick_up_pose, RobotTaskStatus::Arm::ARM_torso);
                RCLCPP_INFO(node->get_logger(), "Moved to PICKUP position.");
                current_state = PICK_BLOCK;
            } catch (const std::exception & e) {
                RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
                running = false;
            } 
        } else if (current_state == PICK_BLOCK) {
              RCLCPP_INFO(node->get_logger(), "Moved to PICKUP position, now picking block...");
              node->GripperControl("OPEN");
              rclcpp::sleep_for(3s);
              node->GripperControl("CLOSE");
              rclcpp::sleep_for(4s);
              current_state = MOVE_TO_PLACE;
        } else if (current_state == MOVE_TO_PLACE) {
          RCLCPP_INFO(node->get_logger(), "Now moving to MOVE TO PLACE position...");
          rclcpp::sleep_for(4s);
          z_target = table_height + gripper_height + (current_layer * block_height);
          
          geometry_msgs::msg::Pose place_pose;
          bool is_even_layer = (current_layer % 2 == 0);
          if (is_even_layer) {
            place_pose.position.x = x_target;
            place_pose.position.y = y_target + (current_block_in_layer * block_spacing); //if currrent_block_in_layer is 0, it will be y_target, if 1, it will be y_target + block_spacing
            place_pose.position.z = z_target;
            RCLCPP_INFO(node->get_logger(), "Placing block in layer %d at coordinates: x:%f y:%f z:%f", current_layer, place_pose.position.x, place_pose.position.y, place_pose.position.z);
            tf2::Quaternion original_quat;
            original_quat.setRPY(0.0, 1.57, 0.0);
            place_pose.orientation.x = original_quat.x();
            place_pose.orientation.y = original_quat.y();
            place_pose.orientation.z = original_quat.z();
            place_pose.orientation.w = original_quat.w();
          } else {
            place_pose.position.x = x_target + (current_block_in_layer * block_spacing);
            place_pose.position.y = y_target;
            place_pose.position.z = z_target;
            RCLCPP_INFO(node->get_logger(), "Placing block in layer %d at coordinates: x:%f y:%f z:%f", current_layer, place_pose.position.x, place_pose.position.y, place_pose.position.z);
            tf2::Quaternion original_quat;
            original_quat.setRPY(0.0, 1.57, 1.57); 
            place_pose.orientation.x = original_quat.x();
            place_pose.orientation.y = original_quat.y();
            place_pose.orientation.z = original_quat.z();
            place_pose.orientation.w = original_quat.w();
          }

          try {
            node->motion_planning_control(place_pose, RobotTaskStatus::Arm::ARM_torso);
            RCLCPP_INFO(node->get_logger(), "Moved to MOVE TO PLACE position.");
            current_state = PLACE_BLOCK;
          } catch (const std::exception & e) {
            RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
            running = false; 
          }

          // //NOT WORKING
          // place_pose.position.z = z_target + 0.4;  //go up a bit to avoid bumping with the tower
          // rclcpp::sleep_for(3s);
          // try {
          //   node->motion_planning_control(place_pose, RobotTaskStatus::Arm::ARM_torso);
          //   RCLCPP_INFO(node->get_logger(), "Moved to MOVE TO PLACE position.");
          //   current_state = PLACE_BLOCK;
          // } catch (const std::exception & e) {
          //   RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
          //   running = false; 
          // }
          // rclcpp::sleep_for(2s);
        } else if (current_state == PLACE_BLOCK) {
          RCLCPP_INFO(node->get_logger(), "Moved to MOVE TO PLACE position, now placing block...");
          node->GripperControl("OPEN");
          rclcpp::sleep_for(2s);
          current_block_in_layer++;

          
          if (current_block_in_layer >= blocks_per_layer) {
            current_block_in_layer = 0;
            current_layer++;
            if (current_layer >= max_layers) {
              RCLCPP_INFO(node->get_logger(), "Finished");
              current_state = COMPLETE;
            } else {
              current_state = MOVE_TO_PICKUP; // Move to next layer
            }
          }
        } else if (current_state == COMPLETE) {
          RCLCPP_INFO(node->get_logger(), "Jenga tower completed");
          running = false; // Exit the loop
        }
      rclcpp::sleep_for(1000ms);



  // TUTORIAL

  //PICK BRICK
  // geometry_msgs::msg::Pose arm_target_pose;
  // arm_target_pose.position.x = 0.64829;
  // arm_target_pose.position.y = -0.050535;
  // arm_target_pose.position.z = 0.93268; 
  // tf2::Quaternion original_quat;
  // original_quat.setRPY(0.0, 1.57, 0.0); // Roll, Pitch, Yaw in radians
  // arm_target_pose.orientation.x = original_quat.x();
  // arm_target_pose.orientation.y = original_quat.y();
  // arm_target_pose.orientation.z = original_quat.z();
  // arm_target_pose.orientation.w = original_quat.w();

  // try {
  //   node->motion_planning_control(arm_target_pose, RobotTaskStatus::Arm::ARM_torso);  
  //   RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
  // } catch (const std::exception & e) {
  //   RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
  // }

  // node->GripperControl("OPEN");


  // rclcpp::sleep_for(5s); 
   
  // node->GripperControl("CLOSE");
  // //PLACE FIRST BRICK
  // arm_target_pose.position.x = 0.64829;
  // arm_target_pose.position.y = -0.050535;
  // arm_target_pose.position.z = 0.7; 
  // original_quat.setRPY(0.0, 1.57, 0.0); // Roll, Pitch, Yaw in radians
  // arm_target_pose.orientation.x = original_quat.x();
  // arm_target_pose.orientation.y = original_quat.y();
  // arm_target_pose.orientation.z = original_quat.z();
  // arm_target_pose.orientation.w = original_quat.w();

  // try {
  //   node->motion_planning_control(arm_target_pose, RobotTaskStatus::Arm::ARM_torso);  
  //   RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
  // } catch (const std::exception & e) {
  //   RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
  // }

  
  // node->GripperControl("CLOSE");
  }



  rclcpp::shutdown();
  return 0;
}



// LEON PIPELINE

//   PPLN_Parameters pipelineParameters = {};

//   // Task start-up
//   PPLN_v_initialize(&pipelineParameters);

//   // Tower building routine
//   while (pipelineParameters.i_towerProgress < pipelineParameters.i_brickQty)
//   {
//     CMD_v_goToReceive(&pipelineParameters);
//     node->GripperControl("OPEN");
//     sleep(1);
//     node->GripperControl("CLOSE");
//     sleep(1);
//     CMD_v_goOverDropPoint(&pipelineParameters);
//     CMD_v_dropBrick(&pipelineParameters);
//     CMD_v_goOverDropPoint(&pipelineParameters);

//     pipelineParameters.i_towerProgress++;
//   }


//   rclcpp::shutdown();
//   return 0;
// }

// void CMD_v_dropBrick(PPLN_Parameters const * const p_parameters)
// {
//   (void) p_parameters;
// }

// void CMD_v_goOverDropPoint(PPLN_Parameters const * const p_parameters)
// {
//   (void) p_parameters;
// }

// void CMD_v_goToReceive(PPLN_Parameters const * const p_parameters)
// {
//   (void) p_parameters;
// }


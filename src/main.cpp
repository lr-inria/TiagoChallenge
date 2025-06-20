/*************************************************
 * Includes
 ************************************************/

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

#include <yaml-cpp/yaml.h>
#include <memory>
#include <chrono>
#include <vector>

#include "pipeline.h"

/*************************************************
 * Main pipeline
 ************************************************/

int main(int argc, char** argv)
{
  PPLN_Parameters pipelineParameters = {};

  // General initialization
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icr_Motionplanning_arms>();
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
  planning_scene_publisher_ = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene",
                                                                                      rclcpp::QoS(1));

  // Task start-up
  PPLN_v_initialize(&pipelineParameters,
                    node);

  // Tower building routine
  while (pipelineParameters.i_towerProgress < pipelineParameters.i_brickQty)
  {
    PPLN_v_runCommand(PPLN_Command::GoToReceive);
    PPLN_v_runCommand(PPLN_Command::OpenGripper);
    PPLN_v_runCommand(PPLN_Command::CloseGripper);
    PPLN_v_runCommand(PPLN_Command::GoOverDropPoint);
    PPLN_v_runCommand(PPLN_Command::DropBrick);
    PPLN_v_runCommand(PPLN_Command::GoOverDropPoint);

    pipelineParameters.i_towerProgress++;
  }

  // Termination
  rclcpp::shutdown();
  return 0;
}

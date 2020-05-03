#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

std::vector<double> random_ik_calculator(ros::NodeHandle node_handle, std::string group_name, std::string eef)
{
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  joint_values[0] = 0.0;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  /* Check whether any joint is outside its joint limits */
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));


  // Setting the some random coordinates for the joints to reach.
  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform(eef);

  // Gives the current pose of the move_group end_effector
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  //Setting the number of tries to calculate inverse kinematics before reporting failed status.
  std::size_t attempts = 10;
  //Timeout for each try.
  double timeout = 0.1;
  //Calculating the inverse kinematics and reporting whether it is successful.
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, attempts, timeout);

  //Updating the joint_values vector with the calculated angles from the inverse kinematics.
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
  //return the joint_angles vector.
  return joint_values;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pepper_ik_calculator");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // Defining three vectors for saving the joint angles resulted from the inverse kinematics for each move_group.
  std::vector<double> right_arm = random_ik_calculator(node_handle, "right_arm", "r_wrist");
  std::vector<double> left_arm = random_ik_calculator(node_handle, "left_arm", "l_wrist");
  std::vector<double> head = random_ik_calculator(node_handle, "head", "Head");
  std::vector<double> waist = random_ik_calculator(node_handle, "waist", "torso");

  static const std::string PLANNING_GROUP = "entire_upper_body";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  //merging the joint_angles calculated for each move_group and sending them to be sent to the robot.
  std::vector<double> overall;
  overall.insert(overall.end(), waist.begin(), waist.end());
  overall.insert(overall.end(), head.begin(), head.end());
  overall.insert(overall.end(), left_arm.begin(), left_arm.end());
  overall.insert(overall.end(), right_arm.begin(), right_arm.end());
  move_group.setJointValueTarget(overall);
  robot_state::RobotState start_state(*move_group.getCurrentState());
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  ros::shutdown();
  return 0;
}

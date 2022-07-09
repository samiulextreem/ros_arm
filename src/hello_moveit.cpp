

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
	{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	auto move_group_node = rclcpp::Node::make_shared("hello_moveit", node_options);

	// We spin up a SingleThreadedExecutor for the current state monitor to get information
	// about the robot's state.
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(move_group_node);
	std::thread([&executor]() { executor.spin(); }).detach();

	// BEGIN_TUTORIAL

	static const std::string PLANNING_GROUP = "panda_arm_hand";
	bool success;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	// The
	// :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
	// class can be easily set up using just the name of the planning group you would like to control and plan for.
	moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

	// We will use the
	// :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
	// class to add and remove collision objects in our "virtual world" scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// Raw pointers are frequently used to refer to the planning group for improved performance.
	const moveit::core::JointModelGroup* joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	// Visualization
	// ^^^^^^^^^^^^^
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "move_group_tutorial",
														move_group.getRobotModel());

	visual_tools.deleteAllMarkers();

	/* Remote control is an introspection tool that allows users to step through a high level script */
	/* via buttons and keyboard shortcuts in RViz */
	visual_tools.loadRemoteControl();

	// RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.0;
	visual_tools.publishText(text_pose, "asparagus_pick&place_demo", rvt::WHITE, rvt::XLARGE);

	// Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	visual_tools.trigger();

	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

	// We can get a list of all the groups in the robot:
	RCLCPP_INFO(LOGGER, "Available Planning Groups:");
	std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
				std::ostream_iterator<std::string>(std::cout, ", "));


	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
	//
	// Next get the current set of joint values for the group.
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	for (int i = 0; i < 8; i++){
		std::cout << joint_group_positions[i]<<"\n";
	}

		// Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
	joint_group_positions[0] = 1.06349; // radians
	joint_group_positions[1] = 0.98092; // radians
	joint_group_positions[2] = 0.436799; // radians
	joint_group_positions[3] = -2.39138; // radians
	joint_group_positions[4] = 2.62228; // radians
	joint_group_positions[5] = 1.64826; // radians
	joint_group_positions[6] = -2.75715; // radians
	joint_group_positions[7] = .5; // radians
	joint_group_positions[8] = .75715; // radians

	move_group.setJointValueTarget(joint_group_positions);

	move_group.setMaxVelocityScalingFactor(0.8);
	move_group.setMaxAccelerationScalingFactor(0.8);

	// success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	// RCLCPP_INFO(LOGGER, "Visualizing joint movement %f (joint space goal) %s", joint_group_positions[2],success ? "" : "FAILED");

	// // Visualize the plan in RViz:
	// visual_tools.deleteAllMarkers();
	// visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
	// visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	// visual_tools.trigger();
	// move_group.execute(my_plan);
	// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

	// Now, let's define a collision object ROS message for the robot to avoid.
	moveit_msgs::msg::CollisionObject collision_object;
	collision_object.header.frame_id = move_group.getPlanningFrame();

	shape_msgs::msg::SolidPrimitive primitive;
	primitive.type = primitive.BOX;

	std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);



	visual_tools.publishText(text_pose, "spawn_asparagus_spear", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();
	

	moveit_msgs::msg::CollisionObject object_to_attach;
	object_to_attach.id = "cylinder1";

	shape_msgs::msg::SolidPrimitive cylinder_primitive;
	cylinder_primitive.type = primitive.CYLINDER;
	cylinder_primitive.dimensions.resize(2);
	cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
	cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.02;

	// We define the frame/pose for this cylinder so that it appears in the gripper.
	object_to_attach.header.frame_id = move_group.getPlanningFrame();
	geometry_msgs::msg::Pose grab_pose;
	grab_pose.orientation.w = .5;
	grab_pose.position.x = -.15;
	grab_pose.position.y = .54;
	grab_pose.position.z = .05;

	// First, we add the object to the world (without using a vector).
	object_to_attach.primitives.push_back(cylinder_primitive);
	object_to_attach.primitive_poses.push_back(grab_pose);
	object_to_attach.operation = object_to_attach.ADD;
	planning_scene_interface.applyCollisionObject(object_to_attach);

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


	joint_group_positions[9] = .5; // radians
	joint_group_positions[10] = .5; // radians
	move_group.setJointValueTarget(joint_group_positions);

	success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	RCLCPP_INFO(LOGGER, "Visualizing joint movement %f (joint space goal) %s", joint_group_positions[2],success ? "" : "FAILED");

	// Visualize the plan in RViz:
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "planning_path_&_moving", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	move_group.execute(my_plan);
	visual_tools.prompt("grip shoulb bo close");
	



	// Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
	// We also need to tell MoveIt that the object is allowed to be in collision with the finger links of the gripper.
	// You could also use applyAttachedCollisionObject to attach an object to the robot directly.
	RCLCPP_INFO(LOGGER, "Attach the object to the robot");
	std::vector<std::string> touch_links;
	touch_links.push_back("panda_rightfinger");
	touch_links.push_back("panda_leftfinger");
	move_group.attachObject(object_to_attach.id, "panda_hand", touch_links);

	visual_tools.publishText(text_pose, "Cut_pick_collect", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

	/* Wait for MoveGroup to receive and process the attached collision object message */


	// We lower the allowed maximum velocity and acceleration to 5% of their maximum.
	// The default values are 10% (0.1).
	// Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
	// or set explicit factors in your code if you need your robot to move faster.
	move_group.setMaxVelocityScalingFactor(0.8);
	move_group.setMaxAccelerationScalingFactor(0.8);
	joint_group_positions[0] = -.784982; // radians
	joint_group_positions[1] = -.784982; // radians

	

	
	move_group.setMaxVelocityScalingFactor(0.8);
	move_group.setMaxAccelerationScalingFactor(0.8);
	move_group.setJointValueTarget(joint_group_positions);

	success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	RCLCPP_INFO(LOGGER, "moving with cylinder at joint positionis%f  %s",joint_group_positions[0], success ? "" : "FAILED");
	move_group.execute(my_plan);

	visual_tools.prompt("Press 'next' to remove object");


	// Detaching and Removing Objects
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Now, let's detach the cylinder from the robot's gripper.
	RCLCPP_INFO(LOGGER, "Detach the object from the robot");
	move_group.detachObject(object_to_attach.id);

	// Show text in RViz of status
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "spear_detached_from_robot", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

	/* Wait for MoveGroup to receive and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

	// Now, let's remove the objects from the world.
	RCLCPP_INFO(LOGGER, "Remove the objects from the world");
	std::vector<std::string> object_ids;
	object_ids.push_back(collision_object.id);
	object_ids.push_back(object_to_attach.id);
	planning_scene_interface.removeCollisionObjects(object_ids);

	// Show text in RViz of status
	visual_tools.publishText(text_pose, "spear_removed", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

	/* Wait for MoveGroup to receive and process the attached collision object message */
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

	// END_TUTORIAL
	visual_tools.deleteAllMarkers();
	visual_tools.trigger();

	rclcpp::shutdown();
	return 0;
}


	// //plan to get to the point of cut start
    // moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
    

    // std::vector<double> joint_group_positions;
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	

	// joint_group_positions[0] = 1.06349; // radians
	// joint_group_positions[1] = 0.98092; // radians
	// joint_group_positions[2] = 0.436799; // radians
	// joint_group_positions[3] = -2.39138; // radians
	// joint_group_positions[4] = 2.62228; // radians
	// joint_group_positions[5] = 1.64826; // radians
	// joint_group_positions[6] = -2.75715; // radians
	// joint_group_positions[7] = .5; // radians
	// joint_group_positions[8] = .75715; // radians

	// std::cout << "new position after setting joint values" << std::endl;
	// for (auto i : joint_group_positions)
	// 	std::cout << i << std::endl;

	// move_group_interface.setJointValueTarget(joint_group_positions);

    // bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);	
	// RCLCPP_INFO(logger, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");
	
	// draw_trajectory_tool_path(my_plan.trajectory_);
	// draw_title("Executing");
	// moveit_visual_tools.trigger();
	// move_group_interface.execute(my_plan);














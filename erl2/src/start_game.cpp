#include "erl2/start_game.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <unistd.h>


int default_pose();

namespace KCL_rosplan {

    StartGameInterface::StartGameInterface(ros::NodeHandle &nh) {
    // here the initialization
    }
	

	 bool StartGameInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        	std::cout << "Adjusting Initial Arm Height to " << msg->parameters[0].value << std::endl;
		
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
		const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
		ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
		
		moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
		kinematic_state->setToDefaultValues();
		const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
		moveit::planning_interface::MoveGroupInterface group("arm");
		const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
		
		group.setStartStateToCurrentState();
		if (msg->parameters[0].value == "h1"){
			group.setNamedTarget("low");
			group.move();
		}
		if (msg->parameters[0].value == "h2"){
			group.setNamedTarget("high");
			group.move();
		}
		
		sleep(5);
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;	
	
	}
}
int main(int argc, char **argv) {
    
	ros::init(argc, argv, "start_game_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
        KCL_rosplan::StartGameInterface start_game(nh);
	start_game.runActionInterface();
	ros::AsyncSpinner spinner(1);
	spinner.start();
	sleep(1.0);
	return 0;
}

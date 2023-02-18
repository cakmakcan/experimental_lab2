#include "exprob_ass2/start_game.h"
#include <unistd.h>

namespace KCL_rosplan {

    StartGameInterface::StartGameInterface(ros::NodeHandle &nh) {
    // here the initialization
    }
	
int default_pose();

	 bool StartGameInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        
        std::cout << "The robot is powering on" << std::endl;	
	default_pose();
	
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
    
int default_pose() {

	moveit::planning_interface::MoveGroupInterface group("arm");
	group.setEndEffectorLink("cluedo_link");
	group.setPoseReferenceFrame("base_link");
	group.setPlannerId("RRTstar");
	group.setNumPlanningAttempts(10);
	group.setPlanningTime(10.0);
	group.allowReplanning(true);
	group.setGoalJointTolerance(0.0001);
	group.setGoalPositionTolerance(0.0001);
	group.setGoalOrientationTolerance(0.001);
	group.setNamedTarget("default");
	group.move();
	sleep(3.0);
	return 0;    
}




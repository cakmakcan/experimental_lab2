#include "erl2/move_arm.h"
#include <unistd.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace KCL_rosplan {
        /**
	 * \brief this is the initialization function of MoveArmInterface class
	 * This function takes as input the node handle and initiliazes an instance of the class
	*/
	MoveArmInterface::MoveArmInterface(ros::NodeHandle &nh) {
			// here the initialization
	}
	
	/**
	 * \brief this is the callback function for the move arm action interface
	 * 
	 * \param msg->parameters[0] defines the current height of the arm (either h1 or h2)
	 * \param msg->parameters[1] defines the desired height to move the arm to (either h1 or h2)
	 *
	 * \return true when the action is complete
	 *
	 * This function checks the given desired height and moves the robot arm to this height using moveit
	 * 
	*/
	
	bool MoveArmInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "Moving Arm from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		
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
		
		if (msg->parameters[1].value == "h1"){
		    group.setNamedTarget("low");
                    group.move();
		    }
		if (msg->parameters[1].value == "h2"){
		    group.setNamedTarget("high");
                    group.move();
		    }
		
		sleep(5);
	       ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
	       return true;
	}
}

int main(int argc, char **argv) {
        ros::init(argc, argv, "MoveArm_rosplan_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::MoveArmInterface move_arm(nh);
        move_arm.runActionInterface();
        ros::AsyncSpinner spinner(1);
        spinner.start();
        sleep(2.0);
        return 0;
    }

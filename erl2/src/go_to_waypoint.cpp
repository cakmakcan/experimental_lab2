#include "erl2/go_to_waypoint.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprob_ass2/TargetAction.h>

namespace KCL_rosplan {

    GoToWaypointInterface::GoToWaypointInterface(ros::NodeHandle &nh) {
        // here the initialization
    }

    bool GoToWaypointInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
    	std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
    	actionlib::SimpleActionClient<erl2::TargetAction> ac("/go_to_point", true);
    	erl2::TargetGoal goal;
    	ac.waitForServer();
    	if(msg->parameters[2].value == "wp1"){
    		goal.x = 2.2;
    		goal.y = 0.0;
    		goal.theta = 0.0;
        }
        else if (msg->parameters[2].value == "wp2"){
		goal.x = 0.0;
		goal.y = 2.2;
		goal.theta = 0.0;
        }
        else if (msg->parameters[2].value == "wp3"){
        	goal.x = - 2.2;
		goal.y = 0.0;
		goal.theta = 0.0;
        }
        else if (msg->parameters[2].value == "wp4"){
		goal.x = 0.0;
		goal.y = - 2.2;
		goal.theta = 0.0;
        }
        else if (msg->parameters[1].value == "wp0"){
		goal.x = 0.0;
		goal.y = 0.0;
		goal.theta = 0.0;
	}
        
	ac.sendGoal(goal);
	ac.waitForResult();
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}
	int main(int argc, char **argv) {
		ros::init(argc, argv, "go_to_waypoint_action", ros::init_options::AnonymousName);
        	ros::NodeHandle nh("~");
        	KCL_rosplan::GoToWaypointInterface go_to_waypoint(nh);
        	go_to_waypoint.runActionInterface();
        	return 0;
        }

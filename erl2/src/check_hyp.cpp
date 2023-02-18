#include "erl2/check_hyp.h"
#include <unistd.h>
#include <string.h>
#include <vector>
#include <typeinfo>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/TargetAction.h>
#include "erl2/HypCheck.h"
#include "erl2/Oracle.h"

ros::ServiceClient client_ontology;
ros::ServiceClient client_oracle;

namespace KCL_rosplan {
       
	CheckHypInterface::CheckHypInterface(ros::NodeHandle &nh) {
	    // here the initialization
	}
	
	bool CheckHypInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		
		erl2::HypCheck h;    
		client_ontology.call(h);
		ROS_INFO("Checking complete hypothesis: ");
		std::vector<std::string> r = h.response.complete_hyps;
		int length_hyp = r.size();
		if (length_hyp == 0){
		    ROS_INFO("No complete hypothesis yet");
		    return false;
		}
		else{
		    ROS_INFO("There are some complete hypotheses");
		    erl2::Oracle o;
		    client_oracle.call(o);
		    std::string correct_ID = std::to_string(o.response.ID);
		    std::cout<<"Correct hypothesis ID: "<<correct_ID<<std::endl;
		    std::cout<<"Current Complete Hypothesis IDs:"<<std::endl;
		    bool correct_flag = false;
		    for(int i=0; i<length_hyp; i++){
		        std::cout<<r[i]<<std::endl;
		        if (r[i] == correct_ID){
		            correct_flag = true;
		        }
		    }
		    if(correct_flag){
		        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		        return true;
		    }
		    else{
		        return false;
		    }
		}
		
		        
		
	}
}

/**
 * \brief this is main function of the node
 * It initializes the node handle, the action interface, and the service clients 
 *
*/
int main(int argc, char **argv) {
	ros::init(argc, argv, "CheckHypCorrect_rosplan_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	client_ontology = nh.serviceClient<erl2::HypCompCheck>("/check_hyp_complete");
	client_oracle = nh.serviceClient<erl2::Oracle>("/oracle_solution");
	KCL_rosplan::CheckHypCorrectInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}

#include "erl2/get_hint.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/TargetAction.h>
#include "erl2/ErlOracle.h"
#include "erl2/Hint.h"
#include <string.h>

ros::ServiceClient client_ontology;
int id;
std::string key;
std::string value;
bool read_flag = true; // flag to mark if the last received hint on the topic was previously checked by the action callback or not. True means: wasn't checked yet and False means was checked


void hint_callback(const erl2::ErlOracle::ConstPtr& msg){
	id = msg->ID;
	key = msg->key;
	value = msg->value;
	ROS_INFO("Hint ID: (%d)", msg->ID);
	ROS_INFO("Hint Key: (%s)", msg->key.c_str());
	ROS_INFO("Hint Value: (%s)", msg->value.c_str());
	read_flag = false;
}

namespace KCL_rosplan {
        
	GetHintInterface::GetHintInterface(ros::NodeHandle &nh) {
			// here the initialization
	}
	
	
	bool GetHintInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		if(read_flag == false){
			read_flag = true;
			if(key != "" && value != "" && key != "-1" && value != "-1"){
				// update the ontology with the new received hint
				erl2::Hint h;
				h.request.ID = std::to_string(id);
				h.request.key = key;
				h.request.value = value;
				std::cout<<"Sent Hint to Ontology"<<std::endl;
				std::cout<<"ID: "<<id<<std::endl;
				std::cout<<"Key: "<<key<<std::endl;
				std::cout<<"Value: "<<value<<std::endl;
				client_ontology.call(h);
				ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());  
			}
			return true;
		}
		
		else{
			return false;
		}
		
	}
}
        
        /**
         * \brief this is main function of the node
         * It initializes the node handle, the action interface, the subscriber, and the service client 
         *
        */
	int main(int argc, char **argv) {
		ros::init(argc, argv, "GetHint_rosplan_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");
		ros::Subscriber hint_sub = nh.subscribe<erl2::ErlOracle>("/oracle_hint",1,hint_callback);
		client_ontology = nh.serviceClient<erl2::Hint>("/add_hint");
		KCL_rosplan::GetHintInterface my_aci(nh);
		my_aci.runActionInterface();
		return 0;
	}

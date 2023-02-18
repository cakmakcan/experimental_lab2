#!/usr/bin/env python

import rospy
import roslib
import time
from std_srvs.srv import Empty, EmptyRequest
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceRequest, DispatchServiceResponse
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from diagnostic_msgs.msg import KeyValue

rosplan_success = False
rosplan_goal = False



def main():

	global rosplan_success, rosplan_goal, armor_interface
	rospy.init_node('plan_manager')
    
	# calling all the rosplan services 
	print ("Calling all rosplan services ..")
	rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
	problem_interface_client = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
	
	rospy.wait_for_service('/rosplan_planner_interface/planning_server')
	planning_interface_client = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
	
	rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
	parsing_interface_client = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
	
	rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
	plan_dispatcher_client = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
	
	rospy.wait_for_service('armor_interface_srv')
	armor_interface_client = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
	
	action_dispatch_sub = rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch',ActionDispatch, clbk_ac_dispatch)
	
	print("Start planning")
    
	rosplan_success = False
 	rosplan_goal = False
	while (rosplan_success == False or rosplan_goal == False):
        
		# print('Problem interface service')
		problem_interface_client()        
		# print('Planning interface service')
		planning_interface_client()
		# print('Parsing interface service')
		parsing_interface_client()
		# print('Plan dispatcher')
		feedback = plan_dispatcher_client()
		print(feedback)
		rosplan_success = feedback.success
		rosplan_goal = feedback.goal_achieved
		print('Replanning')
		
if __name__ == '__main__':
    main()	

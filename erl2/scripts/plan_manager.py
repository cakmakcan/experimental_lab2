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

last_dispatched_action = "" # a variable to store the last dispatched action name to recognize the action that caused the plan failure
hints_received = 0 # a variable to store the number of hints received (because there is a problem with the hints function in PDDL, the hints is reset to 0 every time the get_hint action fails)
update_client = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)

def clbk_ac_dispatch(msg):
	global last_dispatched_action, hints_received
	last_dispatched_action = msg.name
	if msg.name == 'get_hint':
		hints_received = hints_received + 1


def update_waypoint():
	update_request = KnowledgeUpdateServiceRequest()
	update_request.update_type = 0
	update_request.knowledge.knowledge_type = 1
	update_request.knowledge.attribute_name = "unexplored"
	v1 = KeyValue()
	v1.key = "waypoint"
	v1.value = "wp1"
	v2 = KeyValue()
	v2.key = "height"
	v2.value = "h1"
	update_request.knowledge.values = [v1, v2]
	update_client(update_request)
            
	update_request.update_type = 0
	update_request.knowledge.knowledge_type = 1
	update_request.knowledge.attribute_name = "unexplored"
	update_request.knowledge.values[0].key = "waypoint"
	update_request.knowledge.values[0].value = "wp1"
	update_request.knowledge.values[1].key = "height"
	update_request.knowledge.values[1].value = "h2"
	update_client(update_request)
            
	update_request.update_type = 0
	update_request.knowledge.knowledge_type = 1
	update_request.knowledge.attribute_name = "unexplored"
	update_request.knowledge.values[0].key = "waypoint"
	update_request.knowledge.values[0].value = "wp2"
	update_request.knowledge.values[1].key = "height"
	update_request.knowledge.values[1].value = "h1"
	update_client(update_request)
            
	update_request.update_type = 0
	update_request.knowledge.knowledge_type = 1
	update_request.knowledge.attribute_name = "unexplored"
	update_request.knowledge.values[0].key = "waypoint"
	update_request.knowledge.values[0].value = "wp2"
	update_request.knowledge.values[1].key = "height"
	update_request.knowledge.values[1].value = "h2"
	update_client(update_request)
            
	update_request.update_type = 0
	update_request.knowledge.knowledge_type = 1
	update_request.knowledge.attribute_name = "unexplored"
	update_request.knowledge.values[0].key = "waypoint"
	update_request.knowledge.values[0].value = "wp3"
	update_request.knowledge.values[1].key = "height"
	update_request.knowledge.values[1].value = "h1"
	update_client(update_request)
            
	update_request.update_type = 0
	update_request.knowledge.knowledge_type = 1
	update_request.knowledge.attribute_name = "unexplored"
	update_request.knowledge.values[0].key = "waypoint"
	update_request.knowledge.values[0].value = "wp3"
	update_request.knowledge.values[1].key = "height"
	update_request.knowledge.values[1].value = "h2"
	update_client(update_request)
            
	update_request.update_type = 0
	update_request.knowledge.knowledge_type = 1
	update_request.knowledge.attribute_name = "unexplored"
	update_request.knowledge.values[0].key = "waypoint"
	update_request.knowledge.values[0].value = "wp4"
	update_request.knowledge.values[1].key = "height"
	update_request.knowledge.values[1].value = "h1"
	update_client(update_request)
            
	update_request.update_type = 0
	update_request.knowledge.knowledge_type = 1
	update_request.knowledge.attribute_name = "unexplored"
	update_request.knowledge.values[0].key = "waypoint"
	update_request.knowledge.values[0].value = "wp4"
	update_request.knowledge.values[1].key = "height"
	update_request.knowledge.values[1].value = "h2"
	update_client(update_request)
	
	
	
def main():

	global rosplan_success, rosplan_goal, armor_interface
	global last_dispatched_action, hints_received
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

	action_dispatch_sub = rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch',ActionDispatch, clbk_ac_dispatch)

	
	print("Start planning")
	# Call services 
	empty_req = EmptyRequest()
	problem_interface_client(empty_req)
	planning_interface_client(empty_req)
	parsing_interface_client(empty_req)
	req = DispatchServiceRequest()
	goal_success = plan_dispatcher_client(req)
	print("Goal Success Result:", goal_success.success)
	print("Goal Achievement Result:", goal_success.goal_achieved)
	while (goal_success.success == False):
		if last_dispatched_action == "get_hint":
			print("get_hint action failed! Re-Planning!")
			hints_received = hints_received - 1
			print("Hints received", hints_received)
			
			update_request = KnowledgeUpdateServiceRequest()
			update_request.update_type = 0
			update_request.knowledge.knowledge_type = 2
			update_request.knowledge.attribute_name = "hint_percieved"
			update_request.knowledge.function_value = hints_received
			update_client(update_request)
			
			# re-plan
			empty_req = EmptyRequest()
			problem_interface_client(empty_req)
			planning_interface_client(empty_req)
			parsing_interface_client(empty_req)
			req = DispatchServiceRequest()
			goal_success = plan_dispatcher_client(req)
			
		elif last_dispatched_action == "check_hyp":
			print("check_hypothesis_correct action failed!")
			# reset the waypoint function
			update_waypoint()
			
			update_request = KnowledgeUpdateServiceRequest()
			update_request.update_type = 2
			update_request.knowledge.knowledge_type = 1
			update_request.knowledge.attribute_name = "correct_hyp"
			update_client(update_request)
			
			# reset the hints received to zero
			hints_received = 0
            
			# re-plan
			empty_req = EmptyRequest()
			problem_interface_client(empty_req)
			planning_interface_client(empty_req)
			parsing_interface_client(empty_req)
			
			
			req = DispatchServiceRequest()
			goal_success = plan_dispatcher_client(req)
			print("Goal Success Result:", goal_success.success)
			print("Goal Achievement Result:", goal_success.goal_achieved)
		
if __name__ == '__main__':
    main()	

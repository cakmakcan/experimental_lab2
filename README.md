# Repository for the 2nd assignment of the Experimental Robotics Laboratory course

You can start from this package for the implementation of the 2nd assignment of the course. You find here the simulation environment and the oracle node. Please refer to the class slides for more info!The system was implemented and tested on the [docker image](https://hub.docker.com/repository/docker/carms84/exproblab) provided by Prof. Carmine Recchiuto, University of Genova, Italy

|       Author Name: Can Cakmak   | Student ID: 5054534 |   [cakmak1213@gmail.com]   |


## Introduction:

This is a ROS implementation of a robot agent playing a simplified Cluedo Game collecting hints and checking hypotheses. The agent goes randomly to one of four locations in the environment and can move its arm to one of two locations (low, high) to collect hints in the form of *(who, PERSON)*, *(where, PLACE)* and *(what, WEAPON)*. Collected hints are added to the ontology and after 3 hints are collected the agent goes to the center point to check if a correct hypothesis was found yet or not. The agent continues to explore the environment, collect hints, and check hypothesis until it finds the correct hypothesis.


## Component Diagram:

**The knowledge base (ontology)**: this is the OWL ontology representing the current knowledge of the robot agent.

- **ARMOR**: the armor service responsible for connecting with the knowledge base for querying the ontology or updating it. It is fully implemented by [EmaroLab](https://github.com/EmaroLab/armor). In this project, it is mainly used by the ontology server for adding new hypotheses and hints, and querying the individuals of COMPLETE hypothesis class.
 
- **Ontology Server**: a ROS server implemented to handle the communication with AROMR. It can receive two types of service requests: adding hints to the ontology through the services.
 
 - **Simulation Oracle**: a ROS node representing the Oracle of the game. It continuously checks if the robot's end-effector link `cluedo_link` is within the area of one of the specific hint points. If yes, it publishes a random hint on the topic `/oracle_hint` as a `erl2/ErlOracle.h` message. The published hint can contain valid or non-valid values. It also offers a service `/oracle_solution` to send the correct hypothesis ID as a `erl2/Oracle.h` message.

- **ROSPlan**: a ROS package responsible for problem generation, planning, and plan execution given a problem and domain PDDL files.

 - **Moveit**: Robotic manipulation platform responsible for planning and control of the robotic arm's joints to move the end-effector link from one point to the other. Two arm poses were defined for the robot: **h1**: where the end-effector is at height of 0.75 and **h2**: where the end-effector is at z height 1.25
 
 - **Task Manager**: The main node that calls the ROSPlan services: problem generation, planning, parse plan, and plan dispatch. If the plan execution fails, it updates the current knowledge state based on the last dispatched action and re-plan.
 
 - **StartGame Action Interface**: The ROSPlan action interface for the PDDL action `adjust_init_height` responsible for adjusting the initial pose of the robotic arm. The target pose can be either `h1` or `h2`. It calls Moveit to move the end-effector link to the given initial target pose.
 
 - **GoToWaypoint Action Interface**: The ROSPlan action interface for the PDDL action `goto_waypoint` responsible for moving the robot base from one waypoint to another. Waypoints can be one of five: `wp0`: (0 , 0), `wp1`: (2.4 , 0), `wp2`: (0 , 2.4), `wp3`: (-2.4 , 0), and `wp4`: (0 , -2.4). It sends the goal waypoint to the `Go To Point` action server and waits until it is reached.
 
 - **Go To Point Action Server**: a ROS action server that drives the robot towards a given target pose by offering an action service.
 
 - **GetHint Action Interface**: The ROSPlan action interface for the PDDL action `get_hint` responsible for receiving hints from the oracle.
 
 - **MoveArm Action Interface**: The ROSPlan action interface for the PDDL action `move_arm` responsible for moving the robotic arm to a target pose. The target pose can be either `h1` or `h2`. It calls Moveit to move the end-effector link to the given target pose.
 
 - **CheckHyp Action Interface**: The ROSPlan action interface for the PDDL action `check_hypothesis_correct` responsible for checking if one of the collected hypotheses is the correct one or not. It calls the service `/check_hyp_complete` to get the list of complete hypotheses IDs, and it calls the service `/oracle_solution` to get the correct hypothesis ID. It, then, checks if one of the complete hypotheses is the correct one or not.
 
 ![alt text](https://github.com/cakmakcan/experimental_lab2/blob/main/ComponentDiagram.png)
 
 
## Sequence Diagram:

 ![alt text](https://github.com/cakmakcan/experimental_lab2/blob/main/SequenceDiagram.png)
 
 ## Installation and Running Procedures:

To run the program, you need first to install [ARMOR](https://github.com/EmaroLab/armor) in your ROS workspace.

Then, you need to adapt the code in armor_py_api scripts to be in Python3 instead of Python2:
  - add "from armor_api.armor_exceptions import ArmorServiceInternalError, ArmorServiceCallError" in armor_client.py
  - replace all "except rospy.ServiceException, e" with "except rospy.ServiceException as e"
  - modify line 132 of armor_query_client with: "if res.success and len(res.queried_objects) > 1:"

Add the path of the armor modules to your Python path:
```
export PYTHONPATH=$PYTHONPATH:/root/ros_ws/src/armor/armor_py_api/scripts/armor_api/
```
Download this repository to your workspace. Then, build it

```
catkin_make
```

To launch the program, run the following commands in order on four terminal tabs:
- launch ROSplan with the action interfaces: 
```
roslaunch erl2 cluedo.launch
```
- Launch ARMOR:
```
rosrun armor execute it.emarolab.armor.ARMORMainService
```
- Launch the simulation, ontology server, and go_to_point action server:
```
roslaunch erl2 assignment.launch
```
- Run the task manager to start the game:
```
rosrun erl2 plan_manager.py
```
## Result:
**Screenshots for succesfull simulation run:**

1. First action in the plan `StartGame interface` is executed:
![alt text](https://github.com/cakmakcan/experimental_lab2/blob/main/startgamepic.PNG)

2. The action to move to waypoints in the plan `GoToWaypoint interface` is executed:
![alt text](https://github.com/cakmakcan/experimental_lab2/blob/main/gotowaypoint.PNG)

3. The interface to collect hints in the plan `get_hint interface` is executed:
   No hints is collected and re-planing made
![alt text](https://github.com/cakmakcan/experimental_lab2/blob/main/get_hint%20failed.png)

4. The waypoints are explored and replanned until one hint is found.
![alt text](https://github.com/cakmakcan/experimental_lab2/blob/main/hintisfound.png)

5. When the hints are found, `get_hint interface` is run:
![alt text](https://github.com/cakmakcan/experimental_lab2/blob/main/movearm.PNG)

6. the robot collects 3 hints. So, it goes to the center point.
![alt text](https://github.com/cakmakcan/experimental_lab2/blob/main/hypocheck.PNG)

7. Action `CheckHyp interface` is executed. If correct hpy is not found, all the explored waypoints is reset and re-explored again to collect correct hints.

8. When the robot finds the correct hypothesis, the output can be like this: Plan is successful.
![alt text](https://github.com/cakmakcan/experimental_lab2/blob/main/final_result.PNG)

## System Features & Working Assumptions:

- There are four possible (x,y) locations such as (3,0) (0,3) (-3,0) and (0,-3)

- The robot doesn't stand exactly at the previously defined locations. However, it stands at the locations: (2.4,0), (0,2.4), (-2.4,0), (0,-2.4) respectively.

- I defined two possible state of arm high: 1.25 and low: 0.75 to use in moveit.

-  At the begining, startgame interface used to initialized random pose.

- Robot goes the waypoints to get the hints and after visiting waypoints, it goes to center to check hyp is correct or not.

- When checking the correctness of the collected hypotheses, the robot checks all the collected complete hypotheses, whether they are consistent or not.

- The action get_hint fails only when the robot doesn't receive any hint (`cluedo_link` is not within a hint area). Receiving a non-valid hint doesn't cause the action to fail. However, it doesn't add this non-valid hint to the ontology.

- When the action get_hint fails, the plan execution fails and the task manager updates the knowledge base of ROSPlan with the current state (in this case, the number of collected hints and the explored waypoints from the beginning of the round are kept unchanged) and send a re-planning request to ROSPlan.

- When the action check_hyp fails, the plan execution fails and the task manager updates the knowledge base of ROSPlan with the initial state of the system where all waypoints are unexplored and the number of collected hints is 0. Then, the task manager sends a re-planning request to ROSPlan.

- The robot keeps going to waypoints, getting hints, and checking hypotheses until the plan is successful only when the action check_hyp returns true.  

## System's Limitations:
- The robot is constrained to the two defined heights: `h1`: 0.75 and `h2`: 1.25. Changing the hints heights will cause the system to fail.

## Possible Improvements:
- Insted using function in planning, another method can be used because it creates problem.








#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <visualization_msgs/MarkerArray.h>
#include <erl2/ErlOracle.h>
#include <erl2/Oracle.h>

#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <vector>

//In this node is implemented the behavior of the simulation environment. Indeed there is a subscriber to know the position of therobot's links. There is a publisher that allows to visualize the spheric markers in the simulation environment, and those markers corresponds to the place where the hints are recieved. There is another publisher that sends the hint message if and only if the cluedo_link of the robot is closed enough to the spheric marker. And finally there is a service whose aim is to check if the  current ID coincides with the winning ID of the game. This node handles both the generation of hints, both malformed and correct one, that are generated randomly from arrays containing the individuals of the game, and also the creation of the simulated environment composed of spheric markers, that are places in four specific places in the simulation environment and that can be found (randomly) at two different z coordinates. 

// publisher for oracle_hint
ros::Publisher oracle_pub;


// coordinates of the 4 sources of hints in the environment
double markx[4];
double marky[4];
double markz[4];

// x and y coordinates of the last marker visited
double lastmarkx = 0.0;
double lastmarky = 0.0;

// constant arrays containing all the types of individuals
const std::string key[3] = {"who", "what", "where"};

const std::string person[6] = {"missScarlett", "colonelMustard", "mrsWhite", "mrGreen", "mrsPeacock", "profPlum"};
const std::string object[6] = {"candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"};
const std::string place[9] = {"conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"}; 

int uIDs[3]={-1,-1,-1};
// winning ID
int winID = -1;

// vector of oracle_msgs 
std::vector<erl2::ErlOracle> oracle_msgs;


//brief compute the distance between 3D points
//param x: x coordinate of the actual position
//param y: y coordinate of the actual position
//param z: z coordinate of the actual position
//param x1: x coordinate of the target position
//param y1: y coordinate of the target position
//param z1: z coordinate of the target position
//return dist: distance computed between 3D points
//This function computes the distance between the actual 3D position and the 3D position of the target.


double distfromtarget (double x, double y, double z, double x1, double y1, double z1){
	double dist = sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1)+(z-z1)*(z-z1));
	return dist;
	
}

//This function checks if the ID of the current hypothesis is the winning one.
bool oracleService(erl2::Oracle::Request &req, erl2::Oracle::Response &res)
	{
		res.ID = winID;
		return true;
	}


//This is the callback function for the subscriber to the gazebo/link_states topic. If the link of the robot cluedo_link is closed enough to the 'hint generator', so the distance between the cluedo_link and the 'hint generator' is less than a certain threshold, that in this case is of 0,25 m, it generates a random hint that could be a malformed hint, or it could contain a correct hint composed by the ID, the key (so, who, what, where), and the value (so, the individual).

void oracleCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
   for(int i=0; i< msg->name.size(); i++){
	   if (msg->name[i].find("cluedo_link")!= std::string::npos){
		   for(int j=0; j<4;j++){
				if ((distfromtarget(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z, markx[j],marky[j],markz[j])<0.25) && ((lastmarkx !=markx[j]) || (lastmarky != marky[j]))){
				erl2::ErlOracle oracle_msg;
				oracle_msg.ID = rand() % 6;
				// malformed hint
				if(rand()%4==1){
					int a = rand()%5;
					if(a==0){
						oracle_msg.key = "";
						oracle_msg.value = "";
					}
					if (a==1){
						oracle_msg.key="";
						oracle_msg.value=person[rand()%6];
					}
					if (a==2){
						oracle_msg.key="";
						oracle_msg.value=object[rand()%6];
					}
					if (a==3){
						oracle_msg.key="when";
						oracle_msg.value="-1";
					}
					if (a==4){
						oracle_msg.key="who";
						oracle_msg.value="-1";
					}
				}
				// correct hint
				else {
					// select a random key from the array and, depending on the value, select a corresponding random individual
					oracle_msg.key = key[rand()%3];
					bool existing = false;
					for(int k=0; k<oracle_msgs.size();k++){
						if((oracle_msg.ID == oracle_msgs[k].ID)&&(oracle_msg.key == oracle_msgs[k].key)){
							oracle_msg.value = oracle_msgs[k].value;
							existing = true;	
						}
					}
					if ((!existing) || (std::find(std::begin(uIDs), std::end(uIDs), oracle_msg.ID) != std::end(uIDs))){	
						if (oracle_msg.key == "who")
							oracle_msg.value = person[rand()%6];
						if (oracle_msg.key == "what")
							oracle_msg.value = object[rand()%6];
						if (oracle_msg.key == "where")
							oracle_msg.value = place[rand()%9];
						oracle_msgs.push_back(oracle_msg);
					}
				}
				// publish the msg containing the hint
				oracle_pub.publish(oracle_msg);
				// keeping track of the 'hint generator' visited
				lastmarkx = markx[j];
				lastmarky = marky[j];
		   }
		}
	  }
	}
} 

/**
* \brief main function of the simulation.cpp node
* \param argc
* \param argv
* \return 0
*
* This is the main function of the node where the node itself is initialized. Moreover, here are implemented a publisher, in order
* to visualize through spheric markers the sources points where the hints are generated, another publisher which is in charge of 
* publishing the hints of the game when the cluedo_link of the robot approaches the spheric marker, a subscriber to know the links'
* position of the robot and a service that has top check if the ID of the current hypothesis coincides with the winning one. 
* In addition the markers in the environment are imlplemented and displayed in the simulated enviornment, and the winning ID is
* chosen.
*/
int main(int argc, char **argv)
{
// initialization of the node
ros::init(argc, argv, "assignment2");
ros::NodeHandle nh;
// publisher for displaying the markers in the environment
ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "/visualization_marker", 0 );
// publisher for the hints
oracle_pub = nh.advertise<erl2::ErlOracle>( "/oracle_hint", 0 );
// service to check the winning ID
ros::ServiceServer service= nh.advertiseService("/oracle_solution", oracleService);
// subscriber to know the position of the robot's links
ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 10, oracleCallback);
visualization_msgs::MarkerArray markers;
srand (time(NULL));
// these are the possible z coordinates in which the marker sphere can be found 
const double zpos[2] = {0.75, 1.25};
int RandIndex;


visualization_msgs::Marker marker;
marker.header.frame_id = "odom";
marker.header.stamp = ros::Time();

// spheric marker in the position (-3,0)
marker.id = 0;
marker.type = visualization_msgs::Marker::SPHERE;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = -3.0;
markx[0]=-3.0;
marker.pose.position.y = 0.0;
marky[0]=0.0;
RandIndex = rand() % 2;
marker.pose.position.z = zpos[RandIndex];
markz[0]=marker.pose.position.z;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 0.5;
marker.scale.y = 0.5;
marker.scale.z = 0.5;
marker.color.a = 1.0; 
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
markers.markers.push_back(marker);
// spheric marker in the position (3,0)
marker.id = 1;
marker.pose.position.x = 3.0;
markx[1]=3.0;
marker.pose.position.y = 0.0;
marky[1]=0.0;
RandIndex = rand() % 2;
marker.pose.position.z = zpos[RandIndex];
markz[1]=marker.pose.position.z;
markers.markers.push_back(marker);
// spheric marker in the position (0,-3)
marker.id = 2;
marker.pose.position.x = 0.0;
markx[2]=0.0;
marker.pose.position.y = -3.0;
marky[2]=-3.0;
RandIndex = rand() % 2;
marker.pose.position.z = zpos[RandIndex];
markz[2]=marker.pose.position.z;
markers.markers.push_back(marker);
// spheric marker in the position (0,3)
marker.id = 3;
marker.pose.position.x = 0.0;
markx[3]=0.0;
marker.pose.position.y = 3.0;
marky[3]=3.0;
RandIndex = rand() % 2;
marker.pose.position.z = zpos[RandIndex];
markz[3]=marker.pose.position.z;
markers.markers.push_back(marker);

int uid;
for (int i = 0; i < 4; i++){	
	do{
		uid = rand()%6;
		for( int i = 0; i < 3; i++ ){
			if(uid == uIDs[i] ){
					uid = -1;
					break;
				}
			}
		}while(uid == -1);
		
	if(i==3){
		winID = uid;
	}
	else{
    uIDs[i] = uid;
	}
}




while (ros::ok()){
	vis_pub.publish(markers);
	ros::spinOnce();
}

ros::shutdown();

return 0;
}

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <sstream>
#include <iostream> 
#include <string>  
#include <stdlib.h>

using namespace std;

bool state_update = false;

string current_dvrk_state;

string dvrk_states [2] = {
							"DVRK_POSITION_GOAL_CARTESIAN",
							"DVRK_POSITION_CARTESIAN"					
							 };


// PSM robot state publisher/subscriber
ros::Publisher pub_psm_robot_state;
ros::Subscriber sub_psm_robot_state;

// subscribe MTM and publish PSM
ros::Subscriber sub_mtm_robot_cartesian;
ros::Subscriber sub_psm_robot_cartesian;
ros::Publisher pub_psm_robot_position_cartesian;
ros::Publisher pub_psm_robot_goal_cartesian;

// Set DVRK state
void setDVRKState(string state)
{
	std_msgs::String newstate;
	std::stringstream ss;
	ss << state;
	current_dvrk_state=state;
	newstate.data = ss.str();
	pub_psm_robot_state.publish(newstate);
	ROS_INFO("Send to %s state", state.c_str());
}

// Get DVRK state
void getPSM_RobotState_CB(const std_msgs::String::ConstPtr &msg)
{
	if (current_dvrk_state.compare(msg->data) == 0)
	{
		state_update = true;
		ROS_INFO("State set to %s", current_dvrk_state.c_str());
	}
	else 
	{
		state_update = false;
		ROS_INFO("Set to invalid state");
	}
	ROS_INFO("ROBOT STATE: %s", msg->data.c_str());
}
//MTML normalized
float mtmToNormalizedValue(float value,float min, float max){
	 return (value-min)/(max-min);
}	

//PSM normalized
float normalizedValueToPsm(float value, float min, float max){
    return (value * (max-min)) + min;
}

void getMTM_position_cartesian_current_CB(const geometry_msgs::PoseStamped &msg)
{
	if (current_dvrk_state.compare(dvrk_states[0]) == 0){
		geometry_msgs::Pose new_msg;

		new_msg.position.x = 10.0;
		new_msg.position.y = 10.0;
		new_msg.position.z = 10.0;

		new_msg.orientation.x = 10.0;
		new_msg.orientation.y = 10.0;
		new_msg.orientation.z = 10.0;
		new_msg.orientation.w = 10.0;

		pub_psm_robot_goal_cartesian.publish(new_msg);	

	}

	else if (current_dvrk_state.compare(dvrk_states[1]) == 0){
		cout << msg << endl;
		cout << "-----------------------------" << endl;			
		geometry_msgs::Pose new_msg;
		new_msg.position = msg.pose.position;
		new_msg.orientation = msg.pose.orientation;

		  /*
		  MTML   MIN         MAX
		  x:    -0.22        0.35
		  y:    -0.241       0.054
		  z:    -0.35        0.0    
		   
		  PSM1	MIN         MAX
		  x:    -0.22        0.22
		  y:    -0.17        0.16	
		  z:    -0.22       -0.04
		  */
		 
		new_msg.position.x = normalizedValueToPsm(mtmToNormalizedValue(msg.pose.position.x, -0.22, 0.35), -0.22, 0.22);
		new_msg.position.y = normalizedValueToPsm(mtmToNormalizedValue(msg.pose.position.y, -0.241, 0.054), -0.17, 0.16);
		new_msg.position.z = normalizedValueToPsm(mtmToNormalizedValue(msg.pose.position.z, -0.35, 0.0), -0.22, -0.04);
		 
		 
		/*new_msg.position.x = roundf((new_msg.position.x - 0.088773) * 100) / 100;
		new_msg.position.y = roundf((new_msg.position.y - 0.300951) * 100) / 100;
		new_msg.position.z = roundf((new_msg.position.z - 0.052572) * 100) / 100;

		new_msg.orientation.x = roundf((new_msg.orientation.x + 0.61237) * 100) / 100;
		new_msg.orientation.y = roundf((new_msg.orientation.y - 0.612375) * 100) / 100;
		new_msg.orientation.z = roundf((new_msg.orientation.z + 0.353552) * 100) / 100;
		new_msg.orientation.w = roundf((new_msg.orientation.w + 0.353555) * 100) / 100;*/
	
		cout << new_msg << endl;
		cout << "-----------------------------" << endl;
		pub_psm_robot_position_cartesian.publish(new_msg);	

	}			
}

void getPSM_position_cartesian_current_CB(const geometry_msgs::PoseStamped &msg){

	cout << "PSM:" << msg << endl;
	if (current_dvrk_state.compare(dvrk_states[0]) == 0)
	{
		if(msg.pose.position.x == 0.0 &&
			msg.pose.position.y == 0.0 && 
			msg.pose.position.z == 0.0 && 
			msg.pose.orientation.x == 0.0 &&
			msg.pose.orientation.y == 0.0 && 
			msg.pose.orientation.z == 0.0 && 
			msg.pose.orientation.w == 0.0)
		{
			state_update = false;
		}
	}
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "connection");

	ros::NodeHandle nh_;

	pub_psm_robot_state = nh_.advertise<std_msgs::String>("/dvrk/PSM1/set_robot_state", 1000);
	sub_psm_robot_state = nh_.subscribe("/dvrk/PSM1/robot_state", 1000, getPSM_RobotState_CB);	

	ros::Rate loop_rate(20); /*
	//wating dvrk state - goal cartesian checking
	while (ros::ok() && !state_update)
	{
		if (!state_update)
		{
			setDVRKState(dvrk_states[0]);
		}
			   
		ros::spinOnce();
   	loop_rate.sleep();
		ros::Duration(1).sleep(); // sleep for one a second
	}
*/
	pub_psm_robot_position_cartesian = nh_.advertise<geometry_msgs::Pose>("/dvrk/PSM1/set_position_cartesian", 1000);
	pub_psm_robot_goal_cartesian = nh_.advertise<geometry_msgs::Pose>("/dvrk/PSM1/set_position_goal_cartesian", 1000)	;
	sub_mtm_robot_cartesian = nh_.subscribe("/dvrk/MTML/position_cartesian_current", 1000, getMTM_position_cartesian_current_CB);
	sub_psm_robot_cartesian= nh_.subscribe("/dvrk/PSM1/position_cartesian_current", 1000, getPSM_position_cartesian_current_CB);
/*
	// waiting goal cartesian - sets 0.
	while (ros::ok() && state_update)
	{  
		ros::spinOnce();
   	loop_rate.sleep();
		ros::Duration(1).sleep(); // sleep for one a second
	}
*/
	//wating new dvrk state checking
	while (ros::ok() && !state_update)
	{
		if (!state_update)
		{
			setDVRKState(dvrk_states[1]);
		}
			   
		ros::spinOnce();
   	loop_rate.sleep();
		ros::Duration(1).sleep(); // sleep for one a second
	}

	// infinty loop - dvrk position cartesian position update 
	while (ros::ok())
  	{
		ros::spinOnce();
		loop_rate.sleep();
		/*ros::Duration(1).sleep();*/ // sleep for one a second
	}

  	return 0;
}



#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sstream>
#include <signal.h>
#include <ros/xmlrpc_manager.h>
#include <iostream>
#include <termios.h> 
#include <std_msgs/Float64.h>
#include <string>  
#include <stdlib.h>
#include <map>
#include <pthread.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif


using namespace sensor_msgs;
using namespace std;

bool init_state = false;
bool init_position = false;
bool init_get_position = false;

string const dvrk_position_joint_const = "DVRK_POSITION_JOINT";

ros::Publisher pub_rviz_robot_joint;
ros::Subscriber sub_rviz_robot_joint;
   

sensor_msgs::JointState rviz_robot_joint_current;


bool contact = false;
pthread_t myPthread;

enum direction {BACKWARD = 0, RIGHT = 1, FORWARD = 2, LEFT = 3};
direction last_direction = FORWARD;
void set_direction(int offset) {
	int new_dir = int(last_direction) + offset ;
	last_direction = direction(new_dir % 4);
}

bool old_contact = false;

//gazebo
const int command_capacity = 9;
string commands[command_capacity] = {
							"/davinci/joint2_4_position_controller/command",
							"/davinci/joint1_position_controller/command",
							"/davinci/joint3_position_controller/command",
							"/davinci/joint2_3_position_controller/command",
							"/davinci/joint2_2_position_controller/command",
							"/davinci/joint2_position_controller/command",
							"/davinci/joint2_5_position_controller/command",
							"/davinci/joint2_1_position_controller/command",
							"/davinci/joint4_position_controller/command" };

//Adott joint-hoz az utolso elkuldott ertek
std::map<string, double> last_values;

//Összes megnyitott csatorna
std::map<string, ros::Publisher> ros_channels;


//Jelenleg futo publisherek + uzeneteik
std::map<ros::Publisher, std_msgs::Float64> publishers;
typedef std::map<ros::Publisher, std_msgs::Float64>::iterator it_publisher_type;


//jointok initje
void set_initial_params(ros::NodeHandle *nh){

	for (int i = 0; i < command_capacity; i++){
		last_values[commands[i]] == 0.0;
		ros_channels[commands[i]] = (*nh).advertise<std_msgs::Float64> (commands[i], 1000);
	}
}

// RVIZ Publishers and Subscribers
void DVRKInit(ros::Publisher pub_state)
{
	std_msgs::String newstate;
	std::stringstream ss;
	ss << dvrk_position_joint_const;//"DVRK_POSITION_JOINT";
	newstate.data = ss.str();
	pub_state.publish(newstate);
	ROS_INFO("Send to DVRK_POSITION_JOINT state");
}

void rviz_robot_state(const std_msgs::String::ConstPtr &msg)
{
	if (dvrk_position_joint_const.compare(msg->data) == 0)
	{
		init_state = true;
		ROS_INFO("State set to DVRK_POSITION_JOINT");

	}
	else 
	{
		init_state = false;
		ROS_INFO("Set to invalid state");
	}
	//ROS_INFO("ROBOT STATE: %s", msg->data.c_str());
}	

void rviz_joint_cb(const JointState &msg)
{
	rviz_robot_joint_current = msg;
	init_get_position = true;
}


//utkozes detektalasa
void cb(ConstContactsPtr &_msg)
{
   if (_msg->contact_size() !=0 ) {
		contact = true;
    } else {
      contact=false;
    }
}

// publish csatorna + uzenet letrehozasa -> elkuldese a vegtelen ciklusba
void publish_value(string name, double offset) {

	ros::Publisher publisher = ros_channels[name];

	std_msgs::Float64 message; 
	//regi adat + regitol valo elteres
   message.data = last_values[name] += offset;

	//regi felulirasa
	last_values[name] == message.data;

	//"elkuldes" a vegtelen ciklusba
	publishers[publisher] = message;
}


//elore mozgas
void go_forward()
{
	cout<<"forward"<<endl;
	publishers.clear();

	publish_value("/davinci/joint2_2_position_controller/command",  0.01);
	publish_value("/davinci/joint2_3_position_controller/command", -0.01);
}

//hatra mozgas
void go_backward()
{
	cout<<"backward"<<endl;
   publishers.clear();

	publish_value("/davinci/joint2_2_position_controller/command", -0.01);
	publish_value("/davinci/joint2_3_position_controller/command",  0.01);
}

//jobbra mozgas
void go_right()
{
	cout<<"right"<<endl;
	publishers.clear();

	publish_value("/davinci/joint1_position_controller/command", 1.5);
	publish_value("/davinci/joint2_5_position_controller/command", -1.0);
	publish_value("/davinci/joint2_2_position_controller/command", 0.4);
	publish_value("/davinci/joint2_3_position_controller/command", 0.4);
	/*
	publish_value("/davinci/joint1_position_controller/command", 0.05);
	publish_value("/davinci/joint2_3_position_controller/command", 0.00125);	
	*/
}

//balra mozgas

void go_left()
{
	cout<<"left"<<endl;
	publishers.clear();

	publish_value("/davinci/joint1_position_controller/command", -0.05);
	publish_value("/davinci/joint2_3_position_controller/command", -0.00125);
}

void insertion_down()
{
	//Gazebo
	publishers.clear();
	publish_value("/davinci/joint3_position_controller/command",  1.0);

	//RVÍZ,or real robot
	rviz_robot_joint_current.position[2] += 1.0;
	pub_rviz_robot_joint.publish(rviz_robot_joint_current);	
}


void insertion_up()
{
	publishers.clear();
	publish_value("/davinci/joint3_position_controller/command",  -1.0);
}


void move(direction dir){

	switch(dir){
		case FORWARD: 	go_forward();  break;
		case BACKWARD: go_backward(); break;
		case RIGHT: 	go_right();		break;
		case LEFT: 		go_left();     break;
	}
	boost::this_thread::sleep(boost::posix_time::millisec(1500));

}

void update()
{
	if (old_contact!=contact)
	{
		cout<<"update"<<endl;

		bool old = old_contact;
		old_contact=contact;

		if (old == false) {
			set_direction(+1);
			move(last_direction);
		}
		else{
			set_direction(-1);
			move(last_direction);
			set_direction(-1);
			move(last_direction);
		}
	}
	else 
	{
		cout<<"continue"<<endl;
	 	move(last_direction);
	}
}

void *start(void *arg)
{
	cout<<"Enter for Start"<<endl;
	string enter = "";
	getline(cin, enter);
	cout<<"Start"<<endl;
	
	insertion_down();
	boost::this_thread::sleep(boost::posix_time::millisec(8000));
	while (ros::ok()){
		update();
	}
}

int main(int argc, char **argv)
{
  // Load gazebo as a client
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::setupClient(argc, argv);
#else
  gazebo::client::setup(argc, argv);
#endif

  	ros::init(argc, argv, "gazebo_move");

   // Create our node for communication
   gazebo::transport::NodePtr node(new gazebo::transport::Node());
   node->Init();

   // Listen to Gazebo topic
   gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/dvrk_psm/tool_wrist_link/tool_wrist_link_contact/contacts", cb);

	ros::NodeHandle nh_;

	set_initial_params(&nh_);	

	ros::Publisher pub_rviz_robot_state = nh_.advertise<std_msgs::String>("/dvrk/PSM1/set_robot_state", 1000);
	ros::Subscriber sub_rviz_robot_state = nh_.subscribe("/dvrk/PSM1/robot_state", 1000, rviz_robot_state);

   pub_rviz_robot_joint = nh_.advertise<JointState>("/dvrk/PSM1/set_position_joint", 1000);
   sub_rviz_robot_joint = nh_.subscribe("/dvrk/PSM1/state_joint_current", 1000, rviz_joint_cb);

	ros::Rate loop_rate(100); 
	// 
	DVRKInit(pub_rviz_robot_state);

	while (ros::ok() && (!init_state || !init_get_position))
	{
		if (!init_state)
		{
			DVRKInit(pub_rviz_robot_state);
		}
			   
		ros::spinOnce();//Handle events
   	loop_rate.sleep();
		ros::Duration(1).sleep(); // sleep for one a second
	}

	pthread_create(&myPthread, NULL, start, NULL); // start függvény - szál indítása

	//vegtelen ciklus, ha korabban kerult a publishers-be publish, akkor elkuldjuk
	while (ros::ok())
  	{
		gazebo::common::Time::MSleep(20);
		std::map<ros::Publisher, std_msgs::Float64> local_publishers(publishers);
		for(it_publisher_type iterator = local_publishers.begin(); iterator != local_publishers.end(); iterator++) 
		{
			iterator->first.publish(iterator->second);
		}
		ros::spinOnce();
}

#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif
  	return 0;
}



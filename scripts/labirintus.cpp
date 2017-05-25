#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <std_msgs/Float64.h>
#include <string>  
#include <stdlib.h>
#include <map>
#include <array>
#include <pthread.h>
#include <ctime>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif


using namespace sensor_msgs;
using namespace std;

bool contact = false;
pthread_t myPthread;
int penaltyPoints = 0;
time_t t_start, t_end;
bool update_timer_state = false;
bool start_penalty_points = false;
string last_timer_str;
int last_timer_sec;

gazebo::transport::PublisherPtr pub_timer;
gazebo::transport::PublisherPtr pub_score;
gazebo::transport::PublisherPtr pub_popup;

ros::Publisher  pub_robot_joint;
ros::Subscriber sub_robot_joint;

//gazebo
const int command_capacity = 12;

/*
['outer_yaw', 'outer_pitch', 'outer_pitch_1', 'outer_pitch_2', 'outer_pitch_3', 'outer_pitch_4', 'outer_pitch_5', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw', 'jaw_mimic_1', 'jaw_mimic_2']
*/


string commands[command_capacity] = {
							"/davinci/joint1_position_controller/command",
							"/davinci/joint2_position_controller/command",
							"/davinci/joint2_1_position_controller/command",
							"/davinci/joint2_2_position_controller/command",
							"/davinci/joint2_3_position_controller/command",
							"/davinci/joint2_4_position_controller/command",
							"/davinci/joint2_5_position_controller/command",
							"/davinci/joint3_position_controller/command",
							"/davinci/joint4_position_controller/command",
							"/davinci/joint5_position_controller/command",	
							"/davinci/joint6_position_controller/command",
							"/davinci/joint7_position_controller/command",							
							 };

//Összes megnyitott csatorna
std::array<ros::Publisher, command_capacity> gazebo_channels;

//Jelenleg futo publisherek + uzeneteik
std::map<ros::Publisher, std_msgs::Float64> publishers;
typedef std::map<ros::Publisher, std_msgs::Float64>::iterator it_publisher_type;

//jointok initje
void set_initial_params(ros::NodeHandle *nh){

	for (int i = 0; i < command_capacity; i++){
		gazebo_channels[i] = (*nh).advertise<std_msgs::Float64> (commands[i], 1000);
	}
}

// publish csatorna + uzenet letrehozasa -> elkuldese a vegtelen ciklusba
void publish_value(int index, double value) {

	ros::Publisher publisher = gazebo_channels[index];

	std_msgs::Float64 message; 
   message.data = value;

	//"elkuldes" a vegtelen ciklusba
	publishers[publisher] = message;
}

void robot_joint_cb(const JointState::ConstPtr &msg)
{
	for(int i = 0; i < command_capacity; i++){
		publish_value(i, msg->position[i]);
	}	
}


void update_penalty_points(int point)
{
	penaltyPoints = point;
	gazebo::msgs::Any msg1 = gazebo::msgs::ConvertAny(to_string(penaltyPoints));
	pub_score->Publish(msg1);
}

void send_timer_str(string timerStr){
	gazebo::msgs::Any msg1 = gazebo::msgs::ConvertAny(timerStr);
	last_timer_str = timerStr;
	pub_timer->Publish(msg1);
}

void update_timer()
{
   std::ostringstream stream;
	int hour, min, sec;
	stream.str("");

	t_end = clock();

	sec = int(10.0 * (t_end-t_start) / CLOCKS_PER_SEC);
	last_timer_sec = sec;
 	hour = sec / 3600;
  	sec -= hour * 3600;

  	min = sec / 60;
  	sec -= min * 60;

   stream << std::setw(2) << std::setfill('0') << hour << ":";
   stream << std::setw(2) << std::setfill('0') << min << ":";
   stream << std::setw(2) << std::setfill('0') << sec;

	// time adatok küldése  
	send_timer_str(stream.str());
	boost::this_thread::sleep(boost::posix_time::millisec(100));
}

void *start_timer(void *arg)
{
	cout<<"Start"<<endl;
	
   t_start = clock();
	while (ros::ok() && update_timer_state == false){
		update_timer();
	}
}


string total_score(){

int point=50000-int(last_timer_sec/30)-penaltyPoints;

return to_string(point);

}

//utkozes detektalasa
void cb(ConstContactsPtr &_msg)
{
	string collision_name = "dvrk_psm::tool_wrist_link::tool_wrist_link_collision_collision";
	string start_field_collision = "start_field::link_sf::collision_sf";
	string goal_field_collision = "goal_field::link_gf::collision_gf";

	if (_msg->contact_size() !=0 ) {
		for(int i = 0; i < _msg->contact_size(); i++){
			if (_msg->contact(i).collision1() == start_field_collision || _msg->contact(i).collision2() == start_field_collision){
				if(update_timer_state == false){
					update_penalty_points(0);
					send_timer_str("00:00:00");
				}
				update_timer_state = true;
				contact = true;
				return;
			}

		 	else if (_msg->contact(i).collision1() == goal_field_collision || _msg->contact(i).collision2() == goal_field_collision){
				contact = true;
				if (update_timer_state == false){
					update_timer_state = true;
				   start_penalty_points = false;
					string message = "Hiba:" + to_string(penaltyPoints) + "\nIdő:" + last_timer_str + "\n\nÖsszpontszám:" + total_score() + "pont";
					ROS_INFO("%s\n", message.c_str());
					gazebo::msgs::Any msg = gazebo::msgs::ConvertAny(message);
					pub_popup->Publish(msg);
				}
			
				return;
			}

			else if (start_penalty_points == true && (_msg->contact(i).collision1() == collision_name || _msg->contact(i).collision2() == collision_name)){
				contact = true;
				cout<<"Contact!"<<endl;
	 			update_penalty_points(++penaltyPoints);
				return;
			}
		}
	} 
	else {
		if (update_timer_state == true && contact == true)
		{
			update_timer_state = false;
			start_penalty_points = true;
			pthread_create(&myPthread, NULL, start_timer, NULL); // start függvény - szál indítása
		}
	   contact=false;
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

   gazebo::transport::SubscriberPtr sub = node->Subscribe("~/physics/contacts", cb);
	pub_timer = node->Advertise<gazebo::msgs::Any>("~/labyrinth/time");
	pub_score = node->Advertise<gazebo::msgs::Any>("~/labyrinth/score");
	pub_popup = node->Advertise<gazebo::msgs::Any>("~/popup/message");

	ros::NodeHandle nh_;
	set_initial_params(&nh_);
   sub_robot_joint = nh_.subscribe("/dvrk/PSM1/joint_states", 1000, robot_joint_cb);

	ros::Rate loop_rate(100); 
			
	//gazebo::msgs::Any msg = gazebo::msgs::ConvertAny("Hiba: 30 pont\nIdő: 00:00:30\n\nÖsszpontszám: 4800 pont");
	//pub_popup->Publish(msg);
	ROS_INFO("START");
	//vegtelen ciklus, ha korabban kerult a publishers-be publish, akkor elkuldjuk
	while (ros::ok())
  	{
		//gazebo::common::Time::MSleep(5.0);
		std::map<ros::Publisher, std_msgs::Float64> local_publishers(publishers);
		for(it_publisher_type iterator = local_publishers.begin(); iterator != local_publishers.end(); iterator++) 
		{
			iterator->first.publish(iterator->second);
		}
		ros::spinOnce();
		loop_rate.sleep();
		//ros::Duration(1).sleep(); // sleep for one a second

}
	
#if GAZEBO_MAJOR_VERSION < 6
   gazebo::shutdown();
#else
   gazebo::client::shutdown();
#endif

  	return 0;
}



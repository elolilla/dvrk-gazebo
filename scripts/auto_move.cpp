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


//Jelenleg futo publisherek + uzeneteik
std::map<ros::Publisher, std_msgs::Float64> publishers;
typedef std::map<ros::Publisher, std_msgs::Float64>::iterator it_publisher_type;


//jointok initje
void set_initial_params(){
	for (int i = 0; i < command_capacity; i++){
		last_values[commands[i]] == 0.0;
	}
}


//utkozes detektalasa
void cb(ConstContactsPtr &_msg)
{
   if (_msg->contact_size() !=0 ) {
		contact = true;

    } else {
        //NOTHING
    }
}

// publish csatorna + uzenet letrehozasa -> elkuldese a vegtelen ciklusba
void publish_value(string name, double offset) {
	
	static ros::NodeHandle nh_;
	
	ros::Publisher publisher = nh_.advertise<std_msgs::Float64> (name, 1000);

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
	publishers.clear();

	publish_value("/davinci/joint2_2_position_controller/command",  1.0);
	publish_value("/davinci/joint2_3_position_controller/command", -1.0);
}

//hatra mozgas
void go_backward()
{
   publishers.clear();

	publish_value("/davinci/joint2_2_position_controller/command", -1.0);
	publish_value("/davinci/joint2_3_position_controller/command",  1.0);
}

//jobbra mozgas
void go_right()
{

	publishers.clear();

	publish_value("/davinci/joint1_position_controller/command", 0.15);
	publish_value("/davinci/joint2_5_position_controller/command", -1.0);
	publish_value("/davinci/joint2_2_position_controller/command", 0.4);
	publish_value("/davinci/joint2_3_position_controller/command", 0.4);
	
}

//balra mozgas

void go_left()
{
	publishers.clear();

	publish_value("/davinci/joint1_position_controller/command", -0.15);
	publish_value("/davinci/joint2_5_position_controller/command", -1.0);
	publish_value("/davinci/joint2_2_position_controller/command", 0.4);
	publish_value("/davinci/joint2_3_position_controller/command", 0.4);
}

void insertion_down()
{

	publishers.clear();
	publish_value("/davinci/joint3_position_controller/command",  -0.5);
}


void insertion_up()
{

	publishers.clear();
	publish_value("/davinci/joint3_position_controller/command",  0.5);
}


int getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
   newt = oldt;
   newt.c_lflag &= ~(ICANON);                 // disable buffering      
   tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

   int c = getchar();  // read character (non-blocking)

   tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
   return c;
}


int main(int argc, char **argv)
{
	set_initial_params();

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
   gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/dvrk_psm/tool_main_link/tool_main_link_contact/contacts", cb);
	
	//vegtelen ciklus, ha korabban kerult a publishers-be publish, akkor elkuldjuk
	while (ros::ok())
  	{
		gazebo::common::Time::MSleep(20);

		for(it_publisher_type iterator = publishers.begin(); iterator != publishers.end(); iterator++) 
		{
			iterator->first.publish(iterator->second);
		}
		ros::spinOnce();

		int input= getch();

		//Exit
		if(input == 'q'|| input == 'Q' )
		{
				#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif	
				return 0;
		}
		//WASD
		else if (input == 'w'|| input == 'W')
		{
				cout<<"go forward"<<endl;
				go_forward();
		}
		else if (input == 's'|| input == 'S')
		{
				cout<<"go backward"<<endl;
				go_backward();
		}
		else if(input == 'a' || input == 'A' )
		{
			cout<<"go left"<<endl;
			go_left();
		}
		else if(input == 'd' || input == 'D' )
		{
			cout<<"go right"<<endl;
			go_right();
		}
		//
		else if(input == 'x' || input == 'X' )
		{
			cout<<"outer insertion down"<<endl;
			insertion_down();

		}
		else if(input == 'c' || input == 'C' )
		{
			cout<<"outer insertion down"<<endl;
			insertion_up();

		}
	}

#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif
  	return 0;
}



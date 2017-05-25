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

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

using namespace sensor_msgs;
using namespace std;

//1, Add meg a joint számát 1-9
//scanf  ( enter szám, enter. kiírja hogy leüttötte a 13 ast ,scanf ("%d",&i);printf ("Mr. %s , %d years old.\n",str,i);
//nyilak beolvasása hányszor nyomta meg , lenoymás mértékének vizsg getch
//rostopic küldés a megadott jointra
//sirekes küldés után visszatérni a nyilas adat küldésre// beírni h nyomhatja megint a nyilakat
//ha a nyilas bekérésnél van és q nyom lépjen vissza az ad meg a joint számát részhez , ha oda is q ír kilépni az aplikációból

//example: /davinci/joint1_position_controller/command
const string joint_name_prefix = "/davinci/";
const string joint_name_suffix = "/command";
vector<string> joints;

string joint_id = "";
string joint_value = "";


std_msgs::Float64 psm_pose_value;
ros::Publisher pub_psm_joint;

int get_topic_list()
{
   // ros::master::getTopics does not work well
	// https://gist.github.com/bechu/6222399#file-ros_topic_list-cpp
	XmlRpc::XmlRpcValue params("ros_topic_list");
  	XmlRpc::XmlRpcValue results;
  	XmlRpc::XmlRpcValue r;
	int joint_counter = 0;

  	if(ros::master::execute("getTopicTypes", params, results, r, false) == true)
  	{
   	if(results.getType() == XmlRpc::XmlRpcValue::TypeArray)
    	{
      	int32_t i = 2;
     		if(results[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
      	{
        		for (int32_t j = 0; j < results[i].size(); ++j)
        		{
          		if(results[i][j].getType() == XmlRpc::XmlRpcValue::TypeArray)
          		{
            		if(results[i][j].size() == 2)
            		{
              			if(results[i][j][0].getType() == XmlRpc::XmlRpcValue::TypeString && results[i][j][1].getType() == XmlRpc::XmlRpcValue::TypeString)
              			{
								// topic neve
                			string name = static_cast<string>(results[i][j][0]);
								
								//ha túl kicsi neve lenne								
								if(name.length() <= joint_name_suffix.length())
								{
									continue;
								}
								// ha a topic neve /davinci/-val kezdődik && /command-al végződik, akkor ++
								if(name.substr (0,joint_name_prefix.length()) == joint_name_prefix && name.substr (name.length()-joint_name_suffix.length(),joint_name_suffix.length()) == joint_name_suffix)
								{
									joints.push_back(name);
									joint_counter ++;
								}
								
              			}
            		}	
          		}
        		}// end for
      	}
    	}
	}	
	return joint_counter;
}

int get_joint_value()
{
	cout << "Add meg a(z) " << joint_id <<  ". joint új értékét (pl.: 1.0): [q - Back]";
	getline(cin, joint_value);//egész sor beolvasása
	
   //Exit		
   if(joint_value == "q" || joint_value == "Q" )
	{
		return 1;
	}
	else
	{
		double value = strtod(joint_value.c_str(), NULL);//str to double
	
		//betöltés
		psm_pose_value.data = value;
		
		//küldés a csatornára
		pub_psm_joint.publish(psm_pose_value);

		return get_joint_value();
	}
}

int get_joint_id()
{
	int joint_counter = 0;
	vector<string>::iterator joints_iter = joints.begin();   //Tömb aktuális eleme - mutató
   vector<string>::iterator joints_end = joints.end(); 		//Tömb utolsó eleme - mutató
   while(joints_iter != joints_end)
   {	
		cout << joint_counter++ << ".) " << (*joints_iter) << endl;
      joints_iter++;
   }

	cout << "Add meg a joint számát(0-" << joint_counter-1 << "): [q - Quit]";
	getline(cin, joint_id);
	if(joint_id == "q" || joint_id == "Q" )
	{
		return 0;
	}	

	joints_iter = joints.begin();
	advance(joints_iter, atoi(joint_id.c_str())); // Továbbléptetés x-el
	string joint_name = (*joints_iter);
	
	ros::NodeHandle nh_;

	pub_psm_joint = nh_.advertise<std_msgs::Float64> (joint_name, 1000);
	int flag = get_joint_value();	
	if(flag == 1)
	{
		return get_joint_id();
	}
	else {
		return flag;
	}
}

/*
void get_joint_value()
{
	bool flag;

	while (1)
	{ 
		 
  		//cout<<"Enter q to quit"<<endl;
  			
      		int input = getch();   // call your non-blocking input function,-(cin-t helyett)
     
		flag = input == 'q' || input == 'Q';

     	 	if(flag)
		{
			return; 
		}
			
		switch (input)
		{
			case 72:
				// Code for up arrow handling 
				break;
  			}
			case 80:
				// Code for down arrow handling 	
				break;
  			}
			default:
				break;

		}
	}//close while



	
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
*/
void cb(ConstContactsPtr &_msg)
{
   //cout << _msg->contact_size() << endl;

   if (_msg->contact_size() !=0 ) {
        
		  	cout << "Collision: (x: " << _msg->contact(0).wrench().Get(0).body_1_wrench().force().x() << ", y: " << _msg->contact(0).wrench().Get(0).body_1_wrench().force().y() << ", z: " << _msg->contact(0).wrench().Get(0).body_1_wrench().force().z() << ")" << endl;


        //msgForce.x = _msg->contact(0).wrench().Get(0).body_1_wrench().force().x();
        //msgForce.y = _msg->contact(0).wrench().Get(0).body_1_wrench().force().y();
        //msgForce.z = _msg->contact(0).wrench().Get(0).body_1_wrench().force().z();
    } else {
        //cout << "No collision" << endl;
        //msgForce.x = 0;
        //msgForce.y = 0;
        //msgForce.z = 0;
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

   // /gazebo/default/dvrk_psm/outer_insertion_link/outer_insertion_link_contact
   // /gazebo/default/dvrk_psm/outer_insertion_link/outer_insertion_link_contact/contacts

   gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/dvrk_psm/outer_insertion_link/outer_insertion_link_contact/contacts", cb);

	if( get_topic_list() > 0 ) 
	{
		get_joint_id();
	}
	else 
	{
		cout << endl <<"Error - Number of good joints: 0" << endl << endl;
	}
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif
  	return 0;
}



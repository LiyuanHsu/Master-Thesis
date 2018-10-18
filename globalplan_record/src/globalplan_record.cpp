
#include <iostream>
#include <fstream>
#include <cctype>
#include <string>
#include <vector>
#include <complex>
#include <math.h>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"

#include "flatland_msgs/MoveModel.h"
#include "std_srvs/Empty.h"



float goal_x;
float goal_y;

float start_x;
float start_y;

float goal_vector_x;
float goal_vector_y;

float goal_distance;
float goal_angle;
float path_x;
float path_y;
float path_angle;

float pose_x;
float pose_y;


int action;
int path_size;
int path_count = 0;
int iteration = 0;

// it_path
int it_path;

std::list<float> laser;
std::list<float> path_list_x;
std::list<float> path_list_y;
std::list<float>::iterator it_list_x;
std::list<float>::iterator it_list_y;
std::list<float>::iterator it_list_x_old;
std::list<float>::iterator it_list_y_old;
std::list<float>::iterator it_goal_x;
std::list<float>::iterator it_goal_y;

int path_index = 1;

bool set_start = false;
bool set_goal = false;
bool get_path = false;
//bool moving = false;
bool set_pathPtr = false;
bool valid_path = false;

// record csv data
std::string line;
std::string last_line;

// ros publisher
ros::Publisher pub_goal;

// csv file
std::ofstream file;


// pointer for a client
ros::ServiceClient *clientPtr;
ros::ServiceClient *clientPtr_clearCostmaps;

float random_float(float min, float max)    
{    
    return (min + 1) + (((float) rand()) / (float) RAND_MAX) * (max - (min + 1));    
}

int path_to_action(float path_x_, float path_y_)
{
	float path_angle;

	float const angle_inc = 22.5;	

	path_angle = std::atan2(path_y_, path_x_);
	path_angle = path_angle*180/M_PI;

	
	// 8 actions
	if(path_angle >= -22.5 && path_angle < 22.5)	action = 0; 
	if(path_angle >= 22.5 && path_angle < 77.5)		action = 1;
	if(path_angle >= 77.5 && path_angle < 102.5)	action = 2;
	if(path_angle >= 102.5 && path_angle < 157.5)	action = 3;
	if(path_angle >= 157.5 || path_angle < -157.5)	action = 4;
	if(path_angle >= -157.5 && path_angle < -102.5)	action = 5;
	if(path_angle >= -102.5 && path_angle < -77.5)	action = 6;
	if(path_angle >= -77.5 && path_angle < -22.5) 	action = 7;


	/*
	// 16 actions
	if(path_angle >= (angle_inc*0 - 11.25) && path_angle < (angle_inc*0 + 11.25))	action = 0;
	if(path_angle >= (angle_inc*1 - 11.25) && path_angle < (angle_inc*1 + 11.25))	action = 1;
	if(path_angle >= (angle_inc*2 - 11.25) && path_angle < (angle_inc*2 + 11.25))	action = 2;
	if(path_angle >= (angle_inc*3 - 11.25) && path_angle < (angle_inc*3 + 11.25))	action = 3;
	if(path_angle >= (angle_inc*4 - 11.25) && path_angle < (angle_inc*4 + 11.25))	action = 4;
	if(path_angle >= (angle_inc*5 - 11.25) && path_angle < (angle_inc*5 + 11.25))	action = 5;
	if(path_angle >= (angle_inc*6 - 11.25) && path_angle < (angle_inc*6 + 11.25))	action = 6;
	if(path_angle >= (angle_inc*7 - 11.25) && path_angle < (angle_inc*7 + 11.25))	action = 7;

	if(path_angle >= (angle_inc*7 + 11.25) || path_angle < -(angle_inc*7 + 11.25))	action = 8;
	if(path_angle >= -(angle_inc*7 + 11.25) && path_angle < -(angle_inc*7 - 11.25))	action = 9;
	if(path_angle >= -(angle_inc*6 + 11.25) && path_angle < -(angle_inc*6 - 11.25))	action = 10;
	if(path_angle >= -(angle_inc*5 + 11.25) && path_angle < -(angle_inc*5 - 11.25))	action = 11;
	if(path_angle >= -(angle_inc*4 + 11.25) && path_angle < -(angle_inc*4 - 11.25))	action = 12;
	if(path_angle >= -(angle_inc*3 + 11.25) && path_angle < -(angle_inc*3 - 11.25))	action = 13;
	if(path_angle >= -(angle_inc*2 + 11.25) && path_angle < -(angle_inc*2 - 11.25))	action = 14;
	if(path_angle >= -(angle_inc*1 + 11.25) && path_angle < -(angle_inc*1 - 11.25))	action = 15;
	*/
	return action;
}

void publish_goal()
{
	goal_x = random_float(0.5, 8.5);
	goal_y = random_float(0.5, 8.5);	

	//Grid world 
	//goal_x = round(goal_x);
	//goal_y = round(goal_y);

	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "/map";
	msg.pose.position.x = goal_x;
	msg.pose.position.y = goal_y;
	msg.pose.position.z = 0;
	msg.pose.orientation.w = 0;
	msg.pose.orientation.x = 0;
	msg.pose.orientation.y = 0;
	msg.pose.orientation.z = 1;
	pub_goal.publish(msg);

	std::cout << "************************Publish new goal*******************" << '\n';
	ros::Duration(2).sleep();

	get_path = false;

}

void clear_costmaps()
{
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response resp;
	ros::ServiceClient client_clearCostmaps = (ros::ServiceClient)*clientPtr_clearCostmaps;

	bool success = client_clearCostmaps.call(req, resp);
	if(success){
			
		std::cout << "************************Clear Costmaps*******************" << '\n';
	}else{ROS_ERROR_STREAM("Failed to move.");}

	ros::Duration(0.5).sleep();
}

void request_start()
{
	flatland_msgs::MoveModel::Request req;
	flatland_msgs::MoveModel::Response resp; 
	ros::ServiceClient client = (ros::ServiceClient)*clientPtr;

	start_x = random_float(0.5, 8.5);
	start_y = random_float(0.5, 8.5);
		
	//Grid world 
	//start_x = round(start_x);
	//start_y = round(start_y);

	req.pose.x = start_x;
	req.pose.y = start_y;
	req.pose.theta = 0.0; 
	req.name = "bebop103";
	
	
	bool success = client.call(req, resp);
	if(success){
			
		std::cout << "************************Jump to new start*******************" << '\n';
	}else{ROS_ERROR_STREAM("Failed to move.");}
	
	set_start = true;
	path_index = 1;

	clear_costmaps();	
	ros::Duration(2).sleep();

}

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	//std::cout << "Goal_x: " << msg->pose.position.x << ' ' << "Goal_y: " << msg->pose.position.y << '\n';
	
	goal_x = msg->pose.position.x;
	goal_y = msg->pose.position.y;

}

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	laser.clear();
	int size = msg->ranges.size();
	for (unsigned int i = 0; i < msg->ranges.size(); ++i)
	{
		//std::cout << msg->ranges[i] << ' ';
		laser.push_back(msg->ranges[i]);
	}
	//std::cout << '\n';
	//std::cout << "Laser size: " << size << '\n';	
	//std::cout << "Path size: " << path_size << '\n';
	//std::cout << "Goal: " << '(' << goal_x << ',' << goal_y << ')' << '\n';
	//std::cout << "Pose: " << '(' << pose_x << ',' << pose_y << ')' << '\n';

	ros::Duration(0.05).sleep();

	std::list<float>::iterator it_laser;

	flatland_msgs::MoveModel::Request req;
	flatland_msgs::MoveModel::Response resp; 
	ros::ServiceClient client = (ros::ServiceClient)*clientPtr; //dereference the clientPtr
		
	//std::cout << "Ready to call service" << '\n';

if(valid_path == true)
{
	
	if((path_size - path_index) > 60 && get_path == true)
	{
		//file.open ("laser_data.csv");
		advance(it_list_x, 20);
		advance(it_list_y, 20);
		
		std::list<float>::iterator it_list_x_next;
		std::list<float>::iterator it_list_y_next;
		
		it_list_x_next = it_list_x;
		it_list_y_next = it_list_y;
		advance(it_list_x_next, 1);
		advance(it_list_y_next, 1);

		path_index += 20;

		// Normal world
		req.pose.x = *it_list_x;
		req.pose.y = *it_list_y;
		
		//Grid world
		//req.pose.x = round(*it_list_x);
		//req.pose.y = round(*it_list_y);

		req.pose.theta = 0.0; 
		req.name = "bebop103";
		
		goal_vector_x = goal_x - *it_list_x;
		goal_vector_y = goal_y - *it_list_y;
		goal_distance = std::sqrt(std::pow(goal_vector_x, 2) + std::pow(goal_vector_y, 2));
		goal_angle = std::atan2(goal_vector_y, goal_vector_x);
		//goal_angle = goal_angle*180/M_PI;

		path_x = *it_list_x_next - *it_list_x; 
		path_y = *it_list_y_next - *it_list_y;
		action = path_to_action(path_x, path_y);
		
		file << '\n';
		file << path_count;

		for(it_laser=laser.begin(); it_laser!=laser.end(); ++it_laser)			
				file << ',' << *it_laser;
		
		file << ',' << goal_distance << ',' << goal_angle;
		file << ',' << action;

		bool success = client.call(req, resp);
		if(success){
			//ROS_INFO_STREAM("Move a bebop named: ");
			std::cout << "Move to: no." << path_index << " x:" << *it_list_x << ' ' << "y:" << *it_list_y << '\n';
			clear_costmaps();
			ros::Duration(0.1).sleep();	
		}else{
			ROS_ERROR_STREAM("Failed to move.");	
		}
	
		it_list_x_old = it_list_x;
		it_list_y_old = it_list_y;		
		
		//file.close();

	}
	else
	{
		req.pose.x = goal_x;
		req.pose.y = goal_y;
		req.pose.theta = 0.0; 
		req.name = "bebop103";
		
		bool success = client.call(req, resp);
		if(success){
			
			std::cout << "Goooooooooooooooooooooooooooal" << '\n';
			std::cout << "Move to: no." << path_index << " x:" << *it_list_x << ' ' << "y:" << *it_list_y << '\n';	
		}else{
			ROS_ERROR_STREAM("Failed to move.");	
		}
		ros::Duration(2).sleep();
		valid_path = false;

		iteration += 1;
		it_path += 1;
		path_count += 1;
	
		set_start = false;
	
	}
}
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	pose_x = msg->pose.pose.position.x;
	pose_y = msg->pose.pose.position.y;

}

void globalplan_record_cb(const nav_msgs::Path::ConstPtr& msg)
{
if(get_path == false)
{
	get_path = true;
	std::cout << "Into path recording callback" << "\n";
	std::cout << "Path seq: " << msg->header.seq << "\n"; 
	
	bool valid = false;
	int i = 1;

	path_list_x.clear();
	path_list_y.clear();	

	std::vector<geometry_msgs::PoseStamped> data = msg->poses;
	for(std::vector<geometry_msgs::PoseStamped>::iterator it = data.begin(); it != data.end(); ++it)
	{				
		valid = true;
		path_list_x.push_back(it->pose.position.x);
		path_list_y.push_back(it->pose.position.y);
		
		//std::cout << "Point" << i << ": " << "x:" << it->pose.position.x << ' ' << "y:" << it->pose.position.y << '\n'; 		
		i++;
	}
	if(valid == true)
	{ 	
		std::cout << "Get Valid Path" << '\n';
		// set path pointer
		std::cout << "Into set path pointer!!!" << '\n';
		it_list_x = path_list_x.begin();
		it_list_y = path_list_y.begin();
		it_goal_x = path_list_x.end();
		it_goal_y = path_list_y.end();
	
		path_size = data.size();
		if (path_size > 80)
			valid_path = true;
		else
		{
			it_path += 1;
			std::cout << "Short Path" << '\n';
			set_start = false;
			
		}
		//path_count += 1;

	}
	else
	{	
	it_path += 1;
	std::cout << "Unvalid Path" << '\n';
	set_start = false;
	}

	//std::cout << "Path list: ";
	//for(it_list_x=path_list_x.begin(); it_list_x!=path_list_x.end(); ++it_list_x)			
	//	std::cout << ' ' << *it_list_x;

	std::cout << "\n";
	//std::cout << "Path size: " << data.size() << "\n";	
	//std::cout << "List siez: " << path_list_x.size() << "\n";

	//std::advance(it_list_x, 19);
	//std::cout << "20th path vector: " << *it_list_x << '\n';

	//std::cout << "Get path!!!" << '\n';

}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "globalplan_record");
	ros::NodeHandle n;
	
	// csv data
	file.open("laser_data2.csv", std::ios_base::app);
	if(!file)
	{
		file.open("laser_data2.csv");
		file.close();
		file.open("laser_data2.csv", std::ios_base::app);
	}
	
	ros::Subscriber sub_path = n.subscribe("/move_base/NavfnROS/plan", 1, globalplan_record_cb);
		
	// ros subscriber
	//ros::Subscriber sub_goal = n.subscribe("/move_base/current_goal", 1, goal_cb);
	ros::Subscriber sub_laser = n.subscribe("/scan", 1, laser_cb);
	ros::Subscriber sub_odom = n.subscribe("odom", 1, odom_cb);

	// ros publisher
	pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

	// ros client
	ros::ServiceClient client = n.serviceClient<flatland_msgs::MoveModel>("/move_model");
	ros::ServiceClient client_clearCostmaps = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	clientPtr = &client;
	clientPtr_clearCostmaps = &client_clearCostmaps;

	std::cout << "Into recording program" << "\n";	

	
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		if(path_count < 2000)
		{
			std::cout << "it_path:" << it_path << "  " << "path_count:" << path_count << '\n';
			if (set_start == false && valid_path == false)
			{
				//std::cout << "*******INTO request_start********** it_path:" << it_path << '\n';
				request_start();
			}

			if (set_start == true && valid_path == false)
			{
				//std::cout << "*******INTO publish_goal************ it_path:" << it_path << '\n';
				publish_goal();
			}
		
		}
		
		ros::spinOnce();
		loop_rate.sleep();

	}
	
	//client.call(req, resp);

	return 0;
}

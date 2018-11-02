
#include <iostream>
#include <fstream>
#include <sstream>
#include <cctype>
#include <string>
#include <vector>
#include <complex>
#include <math.h>
#include <cmath>
#include <ctime>
//#include <chrono> // for high_resolution_clock

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"

#include "flatland_msgs/MoveModel.h"
#include "flatland_msgs/DeleteModel.h"
#include "flatland_msgs/SpawnModel.h"
#include "std_srvs/Empty.h"

float goal_x;
float goal_y;

float start_x = 2.0;
float start_y = 2.0;

float goal_vector_x;
float goal_vector_y;

float goal_distance;
float goal_angle;
float path_x;
float path_y;
float path_angle;

float pose_x;
float pose_y;

float old_x = 2.0;
float old_y = 2.0;
float last_point_x;
float last_point_y;

int action;
int path_size;
int path_count = 0;
int short_path_count = 0;
int iteration = 0;
int publish_count = 0;

int it_path;

std::list<float> laser;
std::list<float> path_list_x;
std::list<float> path_list_y;
std::list<float> path_list_x_int;
std::list<float> path_list_y_int;
std::list<float>::iterator it_list_x;
std::list<float>::iterator it_list_y;
std::list<float>::iterator it_list_x_int;
std::list<float>::iterator it_list_y_int;
std::list<float>::iterator it_list_x_old;
std::list<float>::iterator it_list_y_old;

int path_index = 1;

bool set_start = false;
bool set_goal = false;
bool get_path = false;
//bool moving = false;
bool set_pathPtr = false;
bool valid_path = false;
bool model_deleted = false;
bool reach_goal = false;
bool trajectory_contain_origin = false;

// record csv data
std::string line;
std::string last_line;

// ros publisher
ros::Publisher pub_goal;

// csv file
std::ofstream file;


// pointer for a client
ros::ServiceClient *clientPtr;
ros::ServiceClient *clientPtr_deleteModel;
ros::ServiceClient *clientPtr_spawnModel;
ros::ServiceClient *clientPtr_clearCostmaps;

template <class T>
std::string toString(const T &value)
{
    std::ostringstream os;
    os << value;
    return os.str();
}

float random_float(float min, float max, long int plus)    
{    
    return (min + 1) + (((float) rand()) / (float) RAND_MAX) * (max - (min + 1));    
}

float half_int(float x)
{
	//return floor(x*2+0.5)/2; // Grid size 0.5m
	return floor(x*5+0.5)/5; // Grid size 0.2m
}

int path_to_action(float path_x_, float path_y_)
{
	float path_angle;

	float const angle_inc = 22.5;	

	path_angle = std::atan2(path_y_, path_x_);
	path_angle = path_angle*180/M_PI;

	
	// 8 actions
	if(path_angle >= -22.5 && path_angle < 22.5)	action = 0; 
	if(path_angle >= 22.5 && path_angle < 77.5)	action = 1;
	if(path_angle >= 77.5 && path_angle < 102.5)	action = 2;
	if(path_angle >= 102.5 && path_angle < 157.5)	action = 3;
	if(path_angle >= 157.5 || path_angle < -157.5)	action = 4;
	if(path_angle >= -157.5 && path_angle < -102.5)	action = 5;
	if(path_angle >= -102.5 && path_angle < -77.5)	action = 6;
	if(path_angle >= -77.5 && path_angle < -22.5) 	action = 7;

	/**
	// 16 actions
	if(path_angle >= (angle_inc*0 - 11.25) && path_angle < (angle_inc*0 + 11.25))	action = 0;
	if(path_angle >= (angle_inc*1 - 11.25) && path_angle < (angle_inc*1 + 11.25))	action = 1;
	if(path_angle >= (angle_inc*2 - 11.25) && path_angle < (angle_inc*2 + 11.25))	action = 2;
	if(path_angle >= (angle_inc*3 - 11.25) && path_angle < (angle_inc*3 + 11.25))	action = 3;
	if(path_angle >= (angle_inc*4 - 11.25) && path_angle < (angle_inc*4 + 11.25))	action = 4;
	if(path_angle >= (angle_inc*5 - 11.25) && path_angle < (angle_inc*5 + 11.25))	action = 5;
	if(path_angle >= (angle_inc*6 - 11.25) && path_angle < (angle_inc*6 + 11.25))	action = 6;
	if(path_angle >= (angle_inc*7 - 11.25) && path_angle < (angle_inc*7 + 11.25))	action = 7

	if(path_angle >= (angle_inc*7 + 11.25) || path_angle < -(angle_inc*7 + 11.25))	action = 8;
	if(path_angle >= -(angle_inc*7 + 11.25) && path_angle < -(angle_inc*7 - 11.25))	action = 9;
	if(path_angle >= -(angle_inc*6 + 11.25) && path_angle < -(angle_inc*6 - 11.25))	action = 10;
	if(path_angle >= -(angle_inc*5 + 11.25) && path_angle < -(angle_inc*5 - 11.25))	action = 11;
	if(path_angle >= -(angle_inc*4 + 11.25) && path_angle < -(angle_inc*4 - 11.25))	action = 12;
	if(path_angle >= -(angle_inc*3 + 11.25) && path_angle < -(angle_inc*3 - 11.25))	action = 13;
	if(path_angle >= -(angle_inc*2 + 11.25) && path_angle < -(angle_inc*2 - 11.25))	action = 14;
	if(path_angle >= -(angle_inc*1 + 11.25) && path_angle < -(angle_inc*1 - 11.25))	action = 15;
	**/
	return action;
}

void clear_costmaps()
{
	std_srvs::Empty::Request req;
	std_srvs::Empty::Response resp;
	ros::ServiceClient client_clearCostmaps = (ros::ServiceClient)*clientPtr_clearCostmaps;

	bool success = client_clearCostmaps.call(req, resp);
	if(success){
		;
		//std::cout << "************************Clear Costmaps*******************" << '\n';
	}else{ROS_ERROR_STREAM("Failed to move.");}

	
}

void delete_model()
{
	// Delete_model
	flatland_msgs::DeleteModel::Request req;
	flatland_msgs::DeleteModel::Response resp;
	ros::ServiceClient client_deleteModel = (ros::ServiceClient)*clientPtr_deleteModel;

	req.name = "bebop103";
	
	bool success = client_deleteModel.call(req, resp);
	if(success)
	{	
		//std::cout << "Model Deleted" << '\n';
		
		model_deleted = true;
	}
}

void spawn_model(float pose_x, float pose_y)
{
	// Spawn_model
	if(model_deleted == true)
	{
		flatland_msgs::SpawnModel::Request req;
		flatland_msgs::SpawnModel::Response resp;
		ros::ServiceClient client_spawnModel = (ros::ServiceClient)*clientPtr_spawnModel;

		req.name = "bebop103";
		req.yaml_path = "/home/lhsu/catkin_ws/src/turtlebot_flatland/robot/turtlebot.model.yaml";
		req.ns = "";
		req.pose.x = pose_x;
		req.pose.y = pose_y;
		req.pose.theta = 0.0;

		bool success = client_spawnModel.call(req, resp);
		if(success)
		{
			//std::cout << "Model Spawned" << '\n';
			
			clear_costmaps();
			
			model_deleted = false;
		}	
	}

}

void recovery()
{
	std::cout << """""""Start Recovery""""""" << '\n';
	publish_count = 0;
	short_path_count = 0;
	delete_model();
	ros::Duration(5).sleep();
	spawn_model(8, 1);
	start_x = 8.0;
	start_y = 1.0;
	ros::Duration(10).sleep();

}

void publish_goal()
{
	//do{
	goal_x = random_float(1, 8, 0);
	goal_y = random_float(1, 8, 100);	

	//Grid world 
	goal_x = half_int(goal_x);
	goal_y = half_int(goal_y);

	//} while ((goal_x == 3 && goal_y == 5) || (goal_x == 5 && goal_y == 3) || (goal_x == 7 && goal_y == 6));

	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "/map";
	msg.pose.position.x = goal_x;
	msg.pose.position.y = goal_y;
	msg.pose.position.z = 0;
	msg.pose.orientation.w = 1;
	msg.pose.orientation.x = 0;
	msg.pose.orientation.y = 0;
	msg.pose.orientation.z = 0;

	
	pub_goal.publish(msg);
	publish_count += 1;
	std::cout << "************************ Publish new goal *******************  " << publish_count << '\n';
	std::cout << "New Goal:(" << goal_x << ',' << goal_y << ')' << '\n';
	ros::Duration(2).sleep();

	get_path = false;

	if (publish_count > 20)
	{
		std::cout << "============ Publish Goal over 20 times =============" << '\n';
		recovery();
	}
}

void move_model(float pose_x, float pose_y)
{
	// Move_model
	flatland_msgs::MoveModel::Request req;
	flatland_msgs::MoveModel::Response resp;
	ros::ServiceClient client = (ros::ServiceClient)*clientPtr;

	req.pose.x = pose_x;
	req.pose.y = pose_y;
	req.pose.theta = 0.0;
	req.name = "bebop103";

	bool success = client.call(req, resp);
	if(success){
		clear_costmaps();
		ros::Duration(0.003).sleep();	
	}

}


void request_start()
{
	//flatland_msgs::MoveModel::Request req;
	//flatland_msgs::MoveModel::Response resp; 
	//ros::ServiceClient client = (ros::ServiceClient)*clientPtr;
	
	


	start_x = random_float(1, 8, 1000);
	start_y = random_float(1, 8, 10000);

	//Grid world
	start_x = half_int(start_x);
	start_y = half_int(start_y);


	//old_x = start_x;
	//old_y = start_y;

	delete_model();
	spawn_model(start_x, start_y);
	std::cout << "************************Jump to new start*******************" << '\n';

	ros::Duration(1).sleep();

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

	//ros::Duration(0.005).sleep();

	std::list<float>::iterator it_laser;

	/**
	flatland_msgs::MoveModel::Request req;
	flatland_msgs::MoveModel::Response resp; 
	ros::ServiceClient client = (ros::ServiceClient)*clientPtr; //dereference the clientPtr
	**/

	//std::cout << "Ready to call service" << '\n';

if(valid_path == true)
{

	//std::cout << "Start at" << path_index << " x:" << old_x << ' ' << "y:" << old_y << '\n';
	

	//for(it_list_x_int=path_list_x_int.begin(); it_list_x_int!=path_list_x_int.end(); ++it_list_x_int)

	//if(old_x != goal_x || old_y != goal_y)	
	{

		file << '\n';
		file << path_count;

		//Record the laser readings		
		for(it_laser=laser.begin(); it_laser!=laser.end(); ++it_laser)			
				file << ',' << *it_laser;

		old_x = *it_list_x_int;
		old_y = *it_list_y_int;		

		it_list_x_int++;
		it_list_y_int++;

		//Calculate and store the goal_distance and goal_angle
		goal_vector_x = goal_x - old_x;
		goal_vector_y = goal_y - old_y;
		goal_distance = std::sqrt(std::pow(goal_vector_x, 2) + std::pow(goal_vector_y, 2));
		goal_angle = std::atan2(goal_vector_y, goal_vector_x);
		file << ',' << goal_distance << ',' << goal_angle;

		//Calculate and store the action
		action = path_to_action((*it_list_x_int - old_x), (*it_list_y_int - old_y)); // Grid world
		file << ',' << action;

		//Move robot to next position (move_model)
		move_model(*it_list_x_int, *it_list_y_int);

		//Calculate and store the action
		//file << ',' << old_x << ',' << old_y;

		std::cout << "At: no." << path_count << " x:" << old_x << ' ' << "y:" << old_y << " action:" << action << '\n';
		
	

	}
	if (*it_list_x_int == goal_x && *it_list_y_int == goal_y)
	{
		// Move to goal (delete_model, spawn_model)
		//delete_model();
		//spawn_model(goal_x, goal_y);
		
		// Move to goal (move_model)
		move_model(goal_x, goal_y);

		std::cout << "GOOOOOOOOOOOOOOOOOOOOAL      ";		
		std::cout << '\n';
		

		start_x = goal_x;
		start_y = goal_y;

		ros::Duration(0.5).sleep();		
		valid_path = false;
		iteration += 1;
		it_path += 1;
		path_count += 1;
		set_start = false;
		set_goal = false;
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

	path_list_x_int.clear();
	path_list_y_int.clear();	

	//path_list_x_int.push_back(start_x);
	//path_list_y_int.push_back(start_y);

	std::vector<geometry_msgs::PoseStamped> data = msg->poses;
	for(std::vector<geometry_msgs::PoseStamped>::iterator it = data.begin(); it != data.end(); ++it)
	{				
		valid = true;
		path_list_x.push_back(it->pose.position.x);
		path_list_y.push_back(it->pose.position.y);
		
		it_list_x_int = path_list_x_int.end();
		it_list_y_int = path_list_y_int.end();
		it_list_x_int--;
		it_list_y_int--;
		
		if (half_int(it->pose.position.x) != *it_list_x_int || half_int(it->pose.position.y) != *it_list_y_int)
		{
			path_list_x_int.push_back(half_int(it->pose.position.x));
			path_list_y_int.push_back(half_int(it->pose.position.y));			
		}

		//std::cout << "Point" << i << ": " << "x:" << it->pose.position.x << ' ' << "y:" << it->pose.position.y << " x_grid:" << half_int(it->pose.position.x) << " y_grid:" << half_int(it->pose.position.y) << '\n'; 		
		i++;
	}
	
	it_list_y_int = path_list_y_int.begin();
	int int_path_count = 0;
	
	//Print start position
	std::cout << "start_x:" << start_x << " start_y:" << start_y << '\n';
	
	for(it_list_x_int=path_list_x_int.begin(); it_list_x_int!=path_list_x_int.end(); ++it_list_x_int)
	{
		int_path_count++;
		std::cout << "Count:" << int_path_count << " Half integer path x:" << *it_list_x_int << " y:" << *it_list_y_int << '\n';
		if(*it_list_x_int == goal_x && *it_list_y_int == goal_y)
		{
			reach_goal = true;
		}	

		if(*it_list_x_int == 0 && *it_list_y_int == 0)
			trajectory_contain_origin = true;		

		std::cout << "reach_goal:" << reach_goal << '\n';

		++it_list_y_int;
	}
	//Print goal position
	std::cout << "goal_x:" << goal_x << " goal_y:" << goal_y << '\n';

	//Find last point
	it_list_x_int = path_list_x_int.end();
	it_list_y_int = path_list_y_int.end();
	it_list_x_int--;
	it_list_y_int--;
	last_point_x = *it_list_x_int;
	last_point_y = *it_list_y_int;


	if(valid == true)
	{ 	
		std::cout << "Get Valid Path" << '\n';
		// set path pointer
		std::cout << "Into set path pointer!!!" << '\n';
		it_list_x = path_list_x.begin();
		it_list_y = path_list_y.begin();
	
		it_list_x_int = path_list_x_int.begin();	
		it_list_y_int = path_list_y_int.begin();

		path_size = data.size();
		if (path_size > 40 && reach_goal == true && trajectory_contain_origin == false)
		{
			valid_path = true;
			set_goal = true;
			publish_count = 0;
			short_path_count = 0;
			reach_goal = false;
		}		
		else
		{
			it_path += 1;
			short_path_count += 1;
			if (path_size <= 40) std::cout << "+++ Short Path +++" << '\n';
			if (reach_goal == false) std::cout << "+++ Goal Mismatch +++" << '\n';
			if (trajectory_contain_origin == true) std::cout << "+++ Traject Contain Origin +++" << '\n'; 

			std::cout << "Path_size:" << path_size << '\n';
		
			set_start = false;
			set_goal = false;
			trajectory_contain_origin = false;
			reach_goal = false;
			
		}
		//path_count += 1;

	}
	else
	{	
	it_path += 1;
	std::cout << "Unvalid Path" << '\n';
	set_start = false;
	set_goal = false;
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
	std::srand(std::time(0));
		
	ros::init(argc, argv, "globalplan_record");
	ros::NodeHandle n;
	
	ros::Subscriber sub_path = n.subscribe("/move_base/NavfnROS/plan", 1, globalplan_record_cb);
		
	// ros subscriber
	//ros::Subscriber sub_goal = n.subscribe("/move_base/current_goal", 1, goal_cb);
	ros::Subscriber sub_laser = n.subscribe("/scan", 1, laser_cb);
	ros::Subscriber sub_odom = n.subscribe("odom", 1, odom_cb);

	// ros publisher
	pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

	// ros client
	ros::ServiceClient client = n.serviceClient<flatland_msgs::MoveModel>("/move_model");
	ros::ServiceClient client_deleteModel = n.serviceClient<flatland_msgs::DeleteModel>("/delete_model");
	ros::ServiceClient client_spawnModel = n.serviceClient<flatland_msgs::SpawnModel>("/spawn_model");
	ros::ServiceClient client_clearCostmaps = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	clientPtr = &client;
	clientPtr_deleteModel = &client_deleteModel;
	clientPtr_spawnModel = &client_spawnModel;
	clientPtr_clearCostmaps = &client_clearCostmaps;

	std::cout << "Into recording program" << "\n";	

	
	ros::Rate loop_rate(100);
	int k = 0;
	while (ros::ok())
	{
		
	while (k<4)
	{
		if (k == 0) file.open("laser_data_0.csv", std::ios_base::app);
		if (k == 1) file.open("laser_data_1.csv", std::ios_base::app);
		if (k == 2) file.open("laser_data_2.csv", std::ios_base::app);
		if (k == 3) file.open("laser_data_3.csv", std::ios_base::app);

		while(path_count < 2000)
		{
			//std::cout << "it_path:" << it_path << "  " << "path_count:" << path_count << '\n';
		
			//if (set_start == false && valid_path == false)
			//{
				//std::cout << "*******INTO request_start********** it_path:" << it_path << '\n';
				//request_start();
			//}
			if (valid_path == false && set_goal == false)
			{
				//std::cout << "*******INTO publish_goal************ it_path:" << it_path << '\n';
				publish_goal();
			}

			if (short_path_count > 20)
			{
				std::cout << "========== Got Short Path over 20 times ==========" << '\n';
				recovery();
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	file.close();
	path_count = 0;
	k++;
		
	}
	
	}
	
	//client.call(req, resp);

	return 0;
}

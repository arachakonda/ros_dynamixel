#include "ros/ros.h"
#include "my_dynamixel_tutorial/GetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

int main(int argc, char **argv){

	ros::init(argc, argv, "read_dynamixel_client");
	ROS_INFO("Spitting out values:");

	ros::NodeHandle nh;
	ros:: ServiceClient client = nh.serviceClient<my_dynamixel_tutorial::GetPosition>("get_position");

	my_dynamixel_tutorial::GetPosition srv;

	srv.request.id = 1; // the Motor ID or MID used to refer to the dynamixel

	while(client.call(srv) && ros::ok()){ // while calling the client is successful and roscore is alive

		usleep(1*1000);
		ROS_INFO("position of the motor is: %d", srv.response.position);
	}

	return 0;


}
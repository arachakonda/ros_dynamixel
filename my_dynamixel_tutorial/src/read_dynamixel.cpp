#include <ros/ros.h>

#include "std_msgs/String.h"
#include "my_dynamixel_tutorial/GetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

#define MID_1 1
#define BAUD 57600
#define DEV_PORT "/dev/ttyUSB0"

PortHandler *portHandler;
PacketHandler *packetHandler;

bool getPresentPositionCallback(my_dynamixel_tutorial::GetPosition::Request &req, 
	my_dynamixel_tutorial::GetPosition::Response &res){

	uint8_t dxl_error = 0;
	int dxl_comm_result = COMM_TX_FAIL;
	int32_t position = 0;

	dxl_comm_result  = packetHandler->read4ByteTxRx(portHandler, (uint8_t)req.id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
	if(dxl_comm_result == COMM_SUCCESS){
		ROS_INFO("getPosition: [ID:%d]->[POSITION:%d]", req.id, position);
		res.position = position;
		return true;
	}
	else{
		ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
		return false;
	}
}

int main(int argc, char ** argv){
	uint8_t dxl_error = 0;
	int dxl_comm_result = COMM_TX_FAIL;

	portHandler = PortHandler::getPortHandler(DEV_PORT); // create port handler object
	packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION); //create packet handler object

	if(!portHandler->openPort()){ // open port
		ROS_ERROR("failed to open port");
		return -1;
	}

	if(!portHandler->setBaudRate(BAUD)){ // set baudrate
		ROS_ERROR("failed to set Baud Rate");
	}

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MID_1, ADDR_TORQUE_ENABLE, 1, &dxl_error);
	if(dxl_comm_result != COMM_SUCCESS){
		ROS_ERROR("failed to enable torque for dynamixel ID %d", MID_1);
		return -1;
	}

	ros::init(argc, argv, "read_dynamixel");
	ros::NodeHandle nh;
	ros::ServiceServer get_position_srv = nh.advertiseService("get_position", getPresentPositionCallback);

	while (ros::ok()){

		usleep(8*1000);
		ros::spin();

	}
	

	portHandler->closePort();
	return 0;


}
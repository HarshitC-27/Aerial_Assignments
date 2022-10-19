#include <ros/ros.h>
#include "smb_highlevel_controller_new/smbHighlevelControllerNew.hpp"
#include "smb_highlevel_controller_new/Scan2.hpp"

int main(int argc, char** argv) {

	// ros::init(argc, argv, "my_node_name")
	ros::init(argc, argv, "smb_highlevel_controller_new");
	// nodeHandle("~") means privete nodeHandle
	ros::NodeHandle nodeHandle("~");

	smb_highlevel_controller_new::Scan2 scan2;

	// create an instance smbHighlevelControllerNew for smb_highlevel_controller_new::smbHighlevelControllerNew
	smb_highlevel_controller_new::smbHighlevelControllerNew smbHighlevelControllerNew(nodeHandle);

	// ros::spin() is a common way of callbacks
	ros::spin();

	return 0;
}
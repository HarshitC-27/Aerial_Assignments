#include <ros/ros.h>
#include "smb_highlevel_controller/SmbHighlevelController.hpp"
#include "smb_highlevel_controller/listener.cpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "smb_highlevel_controller");
  ros::NodeHandle nodeHandle("~");
	smb_highlevel_controller_node::listener scan2;
  smb_highlevel_controller_node::SmbHighlevelController smbHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}

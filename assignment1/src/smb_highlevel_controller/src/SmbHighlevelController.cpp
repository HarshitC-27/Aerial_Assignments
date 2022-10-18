#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
    	nodeHandle.getParam("/SmbHighlevelController/scan_subscriber_topic", scan_subscriber_topic);
  	nodeHandle.getParam("/SmbHighlevelController/scan_subscriber_queue_size", scan_subscriber_queue_size);

	scan_subscriber_ = nodeHandle_.subscribe(scan_subscriber_topic, scan_subscriber_queue_size, &SmbHighlevelController::scanCallback, this);
		ROS_INFO("Successfully launch node");
}
void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan &scan_msg) {
	float smallest_distance = 30;
	std::vector<float> ranges = scan_msg.ranges;
	for (float vec:ranges) {
		if (smallest_distance > vec) smallest_distance = vec;
		/*ROS_INFO_STREAM("ROS_INFO_STREAM laser range values (m) :" << vec);*/
	}	
SmbHighlevelController::~SmbHighlevelController()
{
}

} /* namespace */

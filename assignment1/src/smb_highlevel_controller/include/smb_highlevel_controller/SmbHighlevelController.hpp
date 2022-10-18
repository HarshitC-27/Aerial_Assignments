#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	 listener();
	 virtual ~listener();
	 void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan2);
	// SmbHighlevelController(ros::NodeHandle& nodeHandle);
	// void scanCallback(const sensor_msgs::LaserScan &scan_msg);
	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:
	ros::NodeHandle n;
	ros::Subscriber scan_subscriber_;
	std::string scan_subscriber_topic;
	int scan_subscriber_queue_size;
};

} /* namespace */

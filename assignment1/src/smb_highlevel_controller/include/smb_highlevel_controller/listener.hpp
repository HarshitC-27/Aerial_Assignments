#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

namespace smb_highlevel_controller_node {

	class listener {
	public:
		listener();
		virtual ~listener();
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan2);
	
	private:
		ros::NodeHandle n;
		ros::Publisher scan_pub;
		ros::Subscriber scan_sub;
		

		std::string scan_subscriber_topic;
		int scan_subscriber_queue_size;

	};
}
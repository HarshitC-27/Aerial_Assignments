#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

namespace smb_highlevel_controller_new {
/*	
	Class containing the smb Highlevel Controller (new)
*/
	class smbHighlevelControllerNew {
	public:
		/*
			Constructor.			
		*/
		smbHighlevelControllerNew(ros::NodeHandle& nodeHandle);

		// scanCallback function
		void scanCallback(const sensor_msgs::LaserScan &scan_msg);

		/*
			Destructor.			
		*/
		virtual ~smbHighlevelControllerNew();

	private:
		ros::NodeHandle nodeHandle_;
		ros::Subscriber scan_subscriber_;
		
		std::string scan_subscriber_topic;
		int scan_subscriber_queue_size;
	};



} /*namespace*/
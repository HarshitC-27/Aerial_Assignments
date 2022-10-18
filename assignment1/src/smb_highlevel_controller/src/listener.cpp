// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include "smb_highlevel_controller/SmbHighlevelController.hpp"
// std::float32 data;
// void smallest_dist(std_msgs::String msg)
// {
//     data=msg.range_min;
//     // float32 arr=msg->ranges.c_float32();
//     // float32 min=0;
//     // for(int i=0;i<ranges.length(); i++)
//     // {
//     //     if(min<ranges[i])
//     //     min=ranges[i];
//     // }
//     // float min=msg->range_min;
//     // float min=3.5;
//     ROS_INFO("The smallest distance is %f", data);
// }
// int main(int argc, char* argv[])
// {
//     ros::init(argc, argv, "smb_highlevel_controller");
//     // ros::NodeHandle nodeHandle("~");
//     ros::NodeHandle nh;
//     std:: string topic; int queue_size;
//     nh.getParam("topic", topic);
//     nh.getParam("queue_size", queue_size);
//     // ros:: Subscriber subscriber("/scan", 10, smallest_dist);
//     ros::Subscriber sub=nh.subscribe("/scan", queue_size, smallest_dist);
//     ros::Rate loopRate(30);
//     ros::spin();
//     return 0;
// }

 #include <ros/ros.h>
 #include "smb_highlevel_controller/SmbHighlevelController.hpp"

    namespace smb_highlevel_controller {

	SmbHighlevelController::SmbHighlevelController() {
		scan_pub = n.advertise<sensor_msgs::LaserScan>("/SmbHighlevelController", 1);
		scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &SmbHighlevelController::scanCallback, this);
	}

	void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& SmbHighlevelController) {
		int ranges = SmbHighlevelController->ranges.size();

		sensor_msgs::LaserScan scan;
		// scan.header.stamp = SmbHighlevelController->header.stamp;
		// scan.header.frame_id = SmbHighlevelController->header.frame_id;
		scan.angle_min = SmbHighlevelController->angle_min;
		scan.angle_max = SmbHighlevelController->angle_max;
		scan.angle_increment = SmbHighlevelController->angle_increment;
		scan.time_increment = SmbHighlevelController->time_increment;
		scan.range_min = 0.0;
		scan.range_max = 100.0;

		scan.ranges.resize(ranges);
		for (int i = 0; i < ranges; ++i) {
			scan.ranges[i] = SmbHighlevelController->ranges[i] > 30.0 ? 30.0 : SmbHighlevelController->ranges[i];
			//scan.ranges[i] = SmbHighlevelController->ranges[i];
			ROS_INFO_STREAM("ROS_INFO_STREAM /SmbHighlevelController laser range values (m) :" << scan.ranges[i]);
		}

		ROS_INFO_STREAM("ROS_INFO_STREAM maximum laser distance (m) : " << scan.range_max);


		scan_pub.publish(scan);
	}

	SmbHighlevelController::~SmbHighlevelController() {}

} /* namespace */


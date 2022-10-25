#include <ros/ros.h>
#include "smb_highlevel_controller/smbHighlevelController.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smb_highlevel_controller");
    ros::NodeHandle nodeHandle("~");

    smb_highlevel_controller::smbHighlevelController smbHighlevelController(nodeHandle);
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

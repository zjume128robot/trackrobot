#include"avoidance.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoidance_node");

    ROS_INFO("\n");
    ROS_INFO("****************");
    ROS_INFO("Version:Shenhaoinfo20170306");
    ROS_INFO("****************");
    ROS_INFO("\n");

    avoidance m_avoidance;
    ros::spin();
}

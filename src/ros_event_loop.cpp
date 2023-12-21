#include "ros_event_loop.h"
#include <sensor_msgs/PointCloud2.h>
#include "global_config.hpp"

void update_point_cloud(sensor_msgs::PointCloud2 cloud_msg) {

}

void ros_event_loop(int argc, char** const argv, raii::Window const& window) {
    ros::NodeHandle n;

    // n.subscribe();
    ros::Rate loop_rate(2*global_base_rate);
    bool ros_not_ok_notify_flag = false;  // keeps track of wether a notification of ros failing was sent
    while(!glfwWindowShouldClose(window)){
        if (ros_not_ok_notify_flag && !ros::ok()) {
            puts("ros::ok() == false");
            ros_not_ok_notify_flag = true;
            // todo: retry making a new NodeHandle and reinitialize ros
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
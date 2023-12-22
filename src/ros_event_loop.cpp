#include "ros_event_loop.hpp"
#include <sensor_msgs/PointCloud2.h>
#include "global_config.hpp"

std::condition_variable point_cloud_updated;
std::mutex              point_cloud_mutex;
std::vector<float>      point_cloud_points;  // todo-perf: left-right instead of single lock

#pragma pack(1)
struct CloudPoint {
    float x, y, z;
    uint32_t noise;
    uint16_t intensity;
    uint8_t gray;
};

static
void update_point_cloud(sensor_msgs::PointCloud2 cloud_msg) {
    {
        auto const point_cloud_mutex_guard = std::lock_guard<std::mutex>(point_cloud_mutex);
        point_cloud_points.clear();
        auto const point_array = reinterpret_cast<CloudPoint*>(cloud_msg.data.data());
        size_t const point_count = cloud_msg.data.size()/sizeof(CloudPoint);
        for (size_t i=0; i<point_count; i++) {
            auto const x = point_array[i].x;
            auto const y = point_array[i].y;
            auto const z = point_array[i].z;
            point_cloud_points.push_back(x);
            point_cloud_points.push_back(y);
            point_cloud_points.push_back(z);
        }
    }
    point_cloud_updated.notify_all();
}

void ros_event_loop(int argc, char** const argv, raii::Window const& window) {
    ros::NodeHandle n;
    
    auto transport_hints = ros::TransportHints();
    transport_hints.tcpNoDelay(true);
    auto const subscriber = n.subscribe("/pico_flexx/points", 1024, update_point_cloud, transport_hints);

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
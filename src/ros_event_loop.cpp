#include "ros_event_loop.hpp"
#include <sensor_msgs/PointCloud2.h>
#include "global_config.hpp"
#include <chrono>

std::condition_variable point_cloud_state_updated;
std::mutex              point_cloud_mutex;
// std::vector<float>      point_cloud_points;  // todo-perf: left-right instead of single lock
std::vector<float>      point_cloud_triangles;  // vector of triangles (flattened into floats)
std::atomic<bool>       point_cloud_is_fresh;

#pragma pack(1)
struct CloudPoint {
    float x, y, z;
    uint32_t noise;
    uint16_t intensity;
    uint8_t gray;
};

static
void update_point_cloud(sensor_msgs::PointCloud2 cloud_msg) {
    // computational load of creating a valid set of data is dumped here as an alternative to the main render thread
    size_t const point_count = cloud_msg.data.size()/sizeof(CloudPoint);
    {  // critical section
        // acquire lock
        // auto const point_cloud_mutex_guard = std::lock_guard<std::mutex>(point_cloud_mutex);  // redundant due to atomic bool
        if (point_cloud_is_fresh.load(std::memory_order_acquire)) {
            // fresh data was somehow not yet consumed
            // wait for turn
            using namespace std::chrono_literals;
            auto temp = std::unique_lock(point_cloud_mutex);
            auto constexpr timeout = 100ms;
            point_cloud_state_updated.wait_for(temp, timeout);
            if (point_cloud_is_fresh.load(std::memory_order_acquire))
                return;  // timeout, give up
        }
        // input data
        auto const point_array = reinterpret_cast<CloudPoint*>(cloud_msg.data.data());

        // make guarantees about destination buffer:
        //   - is correct size,
        //   - no pointer invalidation
        point_cloud_triangles.reserve(cloud_msg.data.size()*9);
        point_cloud_triangles.resize( cloud_msg.data.size()*9);
        float* const point_cloud_triangle_ptr = point_cloud_triangles.data();

        for (size_t i=0; i<point_count; i++) {
            auto const x = point_array[i].x;
            auto const y = point_array[i].y;
            auto const z = point_array[i].z;
            // triangle is composed of 3 points, each of 3 floats
            // triangles are flattened into 9 floats each
            size_t const triangle_float_index    = 9*i;
            float* const triangle_vertex_0 = point_cloud_triangle_ptr+triangle_float_index;
            float* const triangle_vertex_1 = triangle_vertex_0 + 3;
            float* const triangle_vertex_2 = triangle_vertex_1 + 3;

            // writing of floats
            triangle_vertex_0[0] = x;
            triangle_vertex_0[1] = y;
            triangle_vertex_0[2] = z;
            triangle_vertex_1[0] = x;
            triangle_vertex_1[1] = y+.1;
            triangle_vertex_1[2] = z;
            triangle_vertex_2[0] = x+.1;
            triangle_vertex_2[1] = y;
            triangle_vertex_2[2] = z;
        }
    }
    point_cloud_is_fresh.store(true, std::memory_order_release);  // store-release is free on x86
    point_cloud_state_updated.notify_all();
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
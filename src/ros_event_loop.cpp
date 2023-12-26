// main logic that deals with ros message passing and data sync

#include "ros_event_loop.hpp"
#include <sensor_msgs/PointCloud2.h>
#include "global_config.hpp"
#include <chrono>

ExposedRenderData shared_render_data;  // a set of variables shared with the main GL render thread

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
        if (shared_render_data.point_cloud_is_fresh.load(std::memory_order_acquire)) {
            // fresh data was somehow not yet consumed
            // wait for turn
            using namespace std::chrono_literals;
            auto temp = std::unique_lock(shared_render_data.point_cloud_mutex);
            auto constexpr timeout = 100ms;
            shared_render_data.point_cloud_state_updated.wait_for(temp, timeout);
            if (shared_render_data.point_cloud_is_fresh.load(std::memory_order_acquire))
                return;  // timeout, give up
        }
        // input data
        auto const point_array = reinterpret_cast<CloudPoint const*>(cloud_msg.data.data());


        auto const height = cloud_msg.height;
        auto const width  = cloud_msg.width;
        auto const row_step = cloud_msg.row_step;

        if (height < 2
         || width < 2 
         || row_step < width*2*sizeof(float))
            return;

        // make guarantees about destination buffer:
        //   - is correct size,
        //   - no pointer invalidation
        // 3 floats/point
        // 3 points/triangle
        // 2 triangles/input point except for top row and left column
        shared_render_data.point_cloud_triangles.resize(3*3*2*(height-1)*(width-1));
        float* const point_cloud_triangle_ptr = shared_render_data.point_cloud_triangles.data();

        for (size_t i=0; i<height-1; i++) {
            for (size_t j=0; j<width-1; j++) {
                auto const top_left  = point_array[i    *width+j  ];
                auto const top_right = point_array[i    *width+j+1];
                auto const bot_left  = point_array[(i+1)*width+j  ];
                auto const bot_right = point_array[(i+1)*width+j+1];
                size_t k = 0;
                // triangle array is 1 element narrower and 1 element shorter than points
                auto const point_cloud_iteration_ptr = &point_cloud_triangle_ptr[3*3*2*((i)*(width-1) + (j))];

                // insert top left tri
                point_cloud_iteration_ptr[k++] = top_left.x;
                point_cloud_iteration_ptr[k++] = top_left.y;
                point_cloud_iteration_ptr[k++] = top_left.z;

                point_cloud_iteration_ptr[k++] = top_right.x;
                point_cloud_iteration_ptr[k++] = top_right.y;
                point_cloud_iteration_ptr[k++] = top_right.z;

                point_cloud_iteration_ptr[k++] = bot_left.x;
                point_cloud_iteration_ptr[k++] = bot_left.y;
                point_cloud_iteration_ptr[k++] = bot_left.z;

                // insert bottom right tri
                point_cloud_iteration_ptr[k++] = top_right.x;
                point_cloud_iteration_ptr[k++] = top_right.y;
                point_cloud_iteration_ptr[k++] = top_right.z;

                point_cloud_iteration_ptr[k++] = bot_right.x;
                point_cloud_iteration_ptr[k++] = bot_right.y;
                point_cloud_iteration_ptr[k++] = bot_right.z;

                point_cloud_iteration_ptr[k++] = bot_left.x;
                point_cloud_iteration_ptr[k++] = bot_left.y;
                point_cloud_iteration_ptr[k++] = bot_left.z;
            }
        }
    }
    shared_render_data.point_cloud_is_fresh.store(true, std::memory_order_release);  // store-release is free on x86
    shared_render_data.point_cloud_state_updated.notify_all();
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
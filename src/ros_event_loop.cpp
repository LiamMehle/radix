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
    {
        // needed for std::cond_var.wait_for()
        using namespace std::chrono_literals;
        std::mutex mtx;
        auto unique_lock = std::unique_lock(mtx);


        // input data
        auto const point_array = reinterpret_cast<CloudPoint const*>(cloud_msg.data.data());

        auto const height = cloud_msg.height;
        auto const width  = cloud_msg.width;
        auto const row_step = cloud_msg.row_step;

        if (height < 2
         || width < 2 
         || row_step < width*2*sizeof(float))
            return;

        size_t const needed_capacity = 3*3*2*static_cast<size_t>(height-1)*static_cast<size_t>(width-1);
        shared_render_data.needed_capacity = needed_capacity;
        
        // edgecase where on failiure to allocate a buffer,
        // or if no buffer was provided, needed capacity is still communicated.
        // this needs to be made visible to other threads (the main thread).
        // No modification of the pointer is intended
        shared_render_data.mapped_buffer.load(std::memory_order_release);

        // guarantee buffer ownership
        while (shared_render_data.mapped_buffer.load(std::memory_order_acquire) == nullptr)
            shared_render_data.update.wait_for(unique_lock, std::chrono::duration(1s));

        // 3 floats/point
        // 3 points/triangle
        // 2 triangles/input point except for top row and left column
        // should be executed only once, but safety is number one priority
        while (shared_render_data.capacity < needed_capacity) {
            shared_render_data.triangle_count = 0;                                          // none written
            shared_render_data.needed_capacity = needed_capacity;                           // known needed capacity

            shared_render_data.mapped_buffer.store(nullptr, std::memory_order_release);     // relinquish onwership of buffer
            shared_render_data.update.notify_all();                                         // contractual obligation to signal update
            shared_render_data.update.wait_for(unique_lock, std::chrono::duration(1s));  // wait for access to mapped buffer
            shared_render_data.mapped_buffer.load(std::memory_order_acquire);               // make sure new data is loaded
        }

        // not guaranteed to have loaded the pointer or related data
        float* const point_cloud_triangle_ptr = shared_render_data.mapped_buffer.load(std::memory_order_acquire);

        for (size_t i=0; i<height-1; i++) {
            for (size_t j=0; j<width-1; j++) {
                auto const top_left  = point_array[i    *width+j  ];
                auto const top_right = point_array[i    *width+j+1];
                auto const bot_left  = point_array[(i+1)*width+j  ];
                auto const bot_right = point_array[(i+1)*width+j+1];
                size_t k = 0;
                // triangle array is 1 element narrower and 1 element shorter than points
                auto const output_width = width-1;
                auto const point_cloud_iteration_ptr = &point_cloud_triangle_ptr[3*3*2*((i)*(output_width) + (j))];

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
        // update shared metadata
        // shared_render_data.capacity is unchanged
        shared_render_data.needed_capacity = needed_capacity;                        // description of work done
        shared_render_data.triangle_count  = needed_capacity/3;                      // description of work done
        shared_render_data.mapped_buffer.store(nullptr, std::memory_order_release);  // relinquish onwership of buffer
        shared_render_data.update.notify_all();                                      // contractual obligation to signal update
    }
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
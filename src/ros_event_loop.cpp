// main logic that deals with ros message passing and data sync

#include "ros_event_loop.hpp"
#include <sensor_msgs/PointCloud2.h>
#include "global_config.hpp"
#include <chrono>

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


        // 3 floats/point
        // 3 points/triangle
        // 2 triangles/input point except for top row and left column
        size_t const needed_capacity = 3*3*2*static_cast<size_t>(height-1)*static_cast<size_t>(width-1);
        
        bool const owns_shared_data = shared_render_data.copy_in_progress.load(std::memory_order_acquire);
        if (!owns_shared_data) {
            shared_render_data.copy_in_progress.store(false, std::memory_order_release);
            return;
        }
        
        auto const inactive_buffer_id = [&]{ return shared_render_data.active_buffer != 0 ? 0 : 1; };
        auto&      owned_buffer = shared_render_data.buffers[inactive_buffer_id()];

        shared_render_data.needed_capacity = needed_capacity;
        if (needed_capacity > owned_buffer.capacity) {
            shared_render_data.copy_in_progress.store(false, std::memory_order_release);
            return;
        }

        float* const point_cloud_triangle_ptr = owned_buffer.mapping;
        if (!point_cloud_triangle_ptr){
            shared_render_data.copy_in_progress.store(false, std::memory_order_release);
            return;
        }

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
                break;
            }
            break;
        }

        float max_distance = 0.f;
        for (size_t i=0; i<height*width; i++) {
            auto const point = point_array[i];
            max_distance = max(max_distance, max(point.x, point.y));
        }

        // update shared metadata
        // shared_render_data.capacity is unchanged                                                         // description of work done
        owned_buffer.zoom = 1/max_distance;
        owned_buffer.triangle_count = needed_capacity/3;                                                    // description of work done
        shared_render_data.copy_in_progress.store(false, std::memory_order_release);                        // relinquish onwership of buffer
        shared_render_data.update.notify_all();                                                             // contractual obligation to signal update
    }
}

void ros_event_loop(int argc, char** const argv) {
    ros::NodeHandle n;
    
    auto transport_hints = ros::TransportHints();
    transport_hints.tcpNoDelay(true);
    auto const subscriber = n.subscribe("/pico_flexx/points", 1024, update_point_cloud, transport_hints);

    ros::spin();
}
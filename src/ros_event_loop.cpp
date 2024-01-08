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

GLFWwindow* offscreen_window;

static
void update_point_cloud(sensor_msgs::PointCloud2 cloud_msg) {
    // computational load of creating a valid set of data is dumped here as an alternative to the main render thread
    size_t const point_count = cloud_msg.data.size()/sizeof(CloudPoint);
    {  // critical section
        // input data
        auto const point_array = reinterpret_cast<CloudPoint const*>(cloud_msg.data.data());


        auto const height = cloud_msg.height;
        auto const width  = cloud_msg.width;
        auto const row_step = cloud_msg.row_step;

        if (height < 2
         || width < 2 
         || row_step < width*2*sizeof(float))
            return;
        
        auto const transient_buffer = shared_render_data.inactive_buffer.load(std::memory_order_consume);
        if (transient_buffer == nullptr)
            return;

        glfwMakeContextCurrent(offscreen_window);
        print_gl_errors("early");
        glBindBuffer(GL_COPY_WRITE_BUFFER, transient_buffer->vbo);
        print_gl_errors("bind buffer");
        // 3 floats/point
        // 3 points/triangle
        // 2 triangles/input point except for top row and left column
        auto const float_count = 3*3*2*((height)*(width-1) + (width));
        glBufferData(GL_COPY_WRITE_BUFFER, float_count*sizeof(float), nullptr, GL_STREAM_DRAW);
        print_gl_errors("buffer storage");
        float* const point_cloud_triangle_ptr = static_cast<float* const>(malloc(float_count * sizeof(float)));
        print_gl_errors("map buffer");
        if (!point_cloud_triangle_ptr)
            return;
        for (size_t i=0; i<height-1; i++) {
            for (size_t j=0; j<width-1; j++) {
                auto const top_left  = point_array[i    *width+j  ];
                auto const top_right = point_array[i    *width+j+1];
                auto const bot_left  = point_array[(i+1)*width+j  ];
                auto const bot_right = point_array[(i+1)*width+j+1];
                size_t k = 0;
                // triangle array is 1 element narrower and 1 element shorter than points
                float* point_cloud_iteration_ptr = &point_cloud_triangle_ptr[3*3*2*((i)*(width-1) + (j))];

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
        transient_buffer->vertex_count = float_count / 3;
        glBufferData(GL_COPY_WRITE_BUFFER, float_count*sizeof(float), point_cloud_triangle_ptr, GL_STREAM_DRAW);
        glFinish();
        free(point_cloud_triangle_ptr);
    }
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
    shared_render_data.inactive_buffer.store(nullptr, std::memory_order_release);
}

void ros_event_loop(int argc, char** const argv, raii::Window const& window) {
    ros::NodeHandle n;
    
    auto transport_hints = ros::TransportHints();
    transport_hints.tcpNoDelay(true);
    auto const subscriber = n.subscribe("/pico_flexx/points", 1024, update_point_cloud, transport_hints);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    offscreen_window = glfwCreateWindow(640, 480, "", NULL, window);

    ros::spin();
}
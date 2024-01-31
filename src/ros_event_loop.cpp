// main logic that deals with ros message passing and data sync

#include "ros_event_loop.hpp"
#include <sensor_msgs/PointCloud2.h>
#include "global_config.hpp"
#include "gl_tools.hpp"
#include <chrono>
#include <GLFW/glfw3.h>


#pragma pack(1)
struct CloudPoint {
    float x, y, z;
    uint32_t noise;
    uint16_t intensity;
    uint8_t gray;
};

GLFWwindow* offscreen_window;

static
void update_point_cloud(sensor_msgs::PointCloud2 const& cloud_msg) {
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
        glBindBuffer(GL_COPY_WRITE_BUFFER, transient_buffer->vbo);
        // 3 floats/point
        // 3 points/triangle
        // 2 triangles/input point except for top row and left column
        auto const vertex_count = 3*2*((height)*(width-1) + (width));
        auto const float_count = 3*vertex_count;

        GLint buffer_size;
        glGetBufferParameteriv(GL_COPY_WRITE_BUFFER, GL_BUFFER_SIZE, &buffer_size);
        if (buffer_size < float_count*sizeof(float))
            // resize buffer
            glBufferData(GL_COPY_WRITE_BUFFER, float_count*sizeof(float), nullptr, GL_STREAM_DRAW);

        // typedef float Vertex[3];
        struct Vertex {float val[3];};

        Vertex* const __restrict buffer = reinterpret_cast<Vertex* __restrict__>(glMapBuffer(GL_COPY_WRITE_BUFFER, GL_WRITE_ONLY));

        for (size_t i=0; i<height-1; i++) {
            for (size_t j=0; j<width-1; j++) {
                auto const row1 = i;
                auto const row2 = i+1;
                auto const col1 = j;
                auto const col2 = j+1;
                auto const to_vertex = [](CloudPoint const& p) -> Vertex { return Vertex { p.x, p.y, p.z }; };
                auto const point_at = [point_array, width](size_t const i, size_t const j) -> CloudPoint const& { return point_array[i * width + j]; };
                auto const top_left  = point_at(row1, col1);
                auto const top_right = point_at(row1, col2);
                auto const bot_left  = point_at(row2, col1);
                auto const bot_right = point_at(row2, col2);

                // triangle array is 1 element narrower and 1 element shorter than points
                Vertex* __restrict__ point_cloud_iteration_ptr = &buffer[3*2*((i)*(width-1) + (j))];

                *point_cloud_iteration_ptr++ = to_vertex(top_left);
                *point_cloud_iteration_ptr++ = to_vertex(top_right);
                *point_cloud_iteration_ptr++ = to_vertex(bot_left);
                *point_cloud_iteration_ptr++ = to_vertex(top_right);
                *point_cloud_iteration_ptr++ = to_vertex(bot_right);
                *point_cloud_iteration_ptr++ = to_vertex(bot_left);
            }
        }
        transient_buffer->vertex_count = vertex_count;
    }
    glUnmapBuffer(GL_COPY_WRITE_BUFFER);
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
    shared_render_data.inactive_buffer.store(nullptr, std::memory_order_release);
}

void ros_event_loop(int argc, char** const argv, GLFWwindow* window) {
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
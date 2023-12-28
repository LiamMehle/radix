// main logic of the program

#include <cstdio>
#include <cstdint>
#include <cassert>
#include <cstring>
#include <string>
#include <vector>
#include <thread>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "gl.h"
#include "gl_tools.hpp"
#include "utils.hpp"
#include "raii.cpp"
#include "global_config.hpp"
#include "ros_event_loop.hpp"

/*
    Base rate at which everything refreshes
    This program is based on polling. This is the base rate, mainly setting the rate a which the display refreshes.
*/

void error_callback(int, const char* err_str) {
    printf("GLFW Error: %s\n", err_str);
}

namespace fs = std::filesystem;
std::string binary_path = fs::canonical("/proc/self/exe").parent_path();
static
int x_error_handler(Display* display, XErrorEvent* event) {
    puts("a");
    return 0;
}

int main(int argc, char** const argv) {
    ros::init(argc, argv, "radix_node");
    // initialize OpenGL
    int status = 0;
    glfwSetErrorCallback(error_callback);
    raii::GLFW glfw{};
    if (glfw != GLFW_TRUE) {
        puts("failed to initialize glfw");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    raii::Window window = raii::Window(WIDTH, HEIGHT, "Radix", nullptr, nullptr);
    if (!static_cast<GLFWwindow*>(window)) {
        puts("failed to create a window");
        return 2;
    }

    // start ros-related thread
    std::thread ros_thread = std::thread([&]{ros_event_loop(argc, argv, window);});

    // continue configuration of window/context
    int frame_buffer_width, frame_buffer_height;
    glfwGetFramebufferSize(window, &frame_buffer_width, &frame_buffer_height);

    glfwMakeContextCurrent(window);
    printf("vendor:   %s\n", reinterpret_cast<char const*>(glGetString(GL_VENDOR)));
    printf("renderer: %s\n", reinterpret_cast<char const*>(glGetString(GL_RENDERER)));

    glewExperimental = GL_TRUE;

    if (glewInit() != GLEW_OK) {
        puts("failed to init glew");
        return 3;
    }

    glViewport(0, 0, frame_buffer_width, frame_buffer_height);

    // todo-perf: compute shader to expand compressed version of the data
    // buffers:
    raii::VAO vao{};
    TriangleBuffer vbo1 {};
    TriangleBuffer vbo2 {};

    glBindVertexArray(vao);                                                       // bind configuration object: remembers the global (buffer) state
        // glBindBuffer(GL_ARRAY_BUFFER, vbo);                                           // bind the buffer to the slot for how it will be used
        // glBufferData(GL_ARRAY_BUFFER, sizeof(verticies), verticies, GL_STATIC_DRAW);  // send data to the gpu (with usage hints)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);                        // configure vbo metadata
        glEnableVertexAttribArray(0);                                                 // enable the config
    glBindVertexArray(0);                                                         // unbinding the buffers (safety?)

    // shaders:
    auto const vertex_shader_path   = binary_path + "/vertex.glsl";
    auto const fragment_shader_path = binary_path + "/fragment.glsl";
    raii::Program program{};
    raii::Shader vertex_shader   = shader_from_file(vertex_shader_path.c_str(),   GL_VERTEX_SHADER);
    raii::Shader fragment_shader = shader_from_file(fragment_shader_path.c_str(), GL_FRAGMENT_SHADER);
    if (!vertex_shader || !fragment_shader)
        return 5;

    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    {
        GLint result;
        glGetProgramiv(program, GL_LINK_STATUS, &result);
        if (!result) {
            std::vector<GLchar> program_log(1024, 0);
            glGetProgramInfoLog(program, program_log.size(), nullptr, program_log.data());
            printf("[error]: %s\n", program_log.data());
            return 6;
        }
        glValidateProgram(program);
        glGetProgramiv(program, GL_LINK_STATUS, &result);
        if (!result) {
            std::vector<GLchar> program_log(1024, 0);
            glGetProgramInfoLog(program, program_log.size(), nullptr, program_log.data());
            printf("[error]: %s\n", program_log.data());
            return 7;
        }
    }

    GLint const x_max_uniform_handle = glGetUniformLocation(program, "x_max");
    GLint const y_max_uniform_handle = glGetUniformLocation(program, "x_max");

    // enable vsync if present:
    set_vsync(true);

    using namespace std::chrono_literals;
    auto constexpr target_frametime = (1000000us)/global_base_rate;
    auto t0 = std::chrono::steady_clock::now();
    auto sleep_duration_adjustment = 0us;
    bool data_is_loaded = false;
    configure_features();
    auto transfer_stream_complete = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    bool active_buffer_index = 0;
    while (!glfwWindowShouldClose(window)) {
        print_gl_errors("top of loop");
        glfwPollEvents();
        print_gl_errors("poll events");
        glBindVertexArray(vao);
        auto&   active_buffer =  active_buffer_index ? vbo1 : vbo2;
        auto& inactive_buffer = !active_buffer_index ? vbo1 : vbo2;
        glBindBuffer(GL_ARRAY_BUFFER,      active_buffer.vbo);
        print_gl_errors("bind active buffer");
        glBindBuffer(GL_COPY_WRITE_BUFFER, inactive_buffer.vbo);
        print_gl_errors("bind inactivebuffer");
        // set up inactive buffer for filling  // persistent mapping is not possible in GL4.1, so a little more work is needed
        if (shared_render_data.mapped_buffer.load(std::memory_order_acquire) == nullptr) {  // ownership is given
            GLint inactive_buffer_is_mapped = 0;
            glGetBufferParameteriv(GL_COPY_WRITE_BUFFER, GL_BUFFER_MAPPED, &inactive_buffer_is_mapped);
            if (inactive_buffer_is_mapped)
                glUnmapBuffer(GL_COPY_WRITE_BUFFER);  // buffer should not be mapped while manipulated
            print_gl_errors("unmap buffer");
            auto const new_capacity = max(static_cast<size_t>(4096), shared_render_data.needed_capacity);
            auto const new_buffer_size_in_bytes = new_capacity*sizeof(decltype(*shared_render_data.mapped_buffer));
            // we have ownership, was it successful?
            if (shared_render_data.needed_capacity > shared_render_data.capacity) {
                // having sole access to the GL, we reallocate needed capacity and re-release
                // buffer was too small, no other work is needed
            } else {
                // swap buffers, set up (newly) inactive buffer mapping and noitfy
                active_buffer_index = !active_buffer_index;
                swap(active_buffer, inactive_buffer);
                glBindBuffer(GL_ARRAY_BUFFER,      active_buffer.vbo);
                glBindBuffer(GL_COPY_WRITE_BUFFER, inactive_buffer.vbo);

                // current buffer may be too big, invalidate and reallocated new inactive buffer
            }
            glBufferData(GL_COPY_WRITE_BUFFER, new_buffer_size_in_bytes*2, nullptr, GL_STREAM_DRAW);
            // release
            print_gl_errors("pre-mapping");
            auto const mapping = static_cast<float*>(glMapBufferRange(GL_COPY_WRITE_BUFFER, static_cast<GLintptr>(0), new_buffer_size_in_bytes,
            // access:
                  GL_MAP_WRITE_BIT              // only write is needed
                | GL_MAP_COHERENT_BIT           // optimized for iGP unified memory, will still release memory
                | GL_MAP_INVALIDATE_RANGE_BIT   // the buffer was just allocated, this flag should break any false data dependancies
            ));
            print_gl_errors("post-mapping");
            puts("-----------------------------------------------------------");
                shared_render_data.capacity = new_capacity;
            shared_render_data.needed_capacity = 0;
            shared_render_data.triangle_count = 0;
            shared_render_data.mapped_buffer.store(mapping, std::memory_order_release);
            shared_render_data.update.notify_all();
        }

        // ----------- drawing -----------
        // clear screen
        glClearColor(.1f, .1f, .1f, 5.f);
        glClear(GL_COLOR_BUFFER_BIT);   
        // pass data to the gpu to be able to perform compute    
        glUniform1f(x_max_uniform_handle, 0);
        glUniform1f(y_max_uniform_handle, 0);
        glDrawArrays(GL_TRIANGLES, 0, active_buffer.triangle_count);  // draw call
        glfwSwapBuffers(window);
        // logic time end
        auto const t1 = std::chrono::steady_clock::now();
        auto const logic_time = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0);
        // adjustment is divided by 2 as a heuristic (avoid large +/- swings, effectively P from PID with factor of .5)
        auto const time_to_sleep_for = target_frametime - logic_time + (sleep_duration_adjustment/2);
        // max prevents underflow
        auto const safe_time_to_sleep_for = max(static_cast<int64_t>(0), std::chrono::duration_cast<std::chrono::microseconds>(time_to_sleep_for).count());
        auto const clamped_time_to_sleep_for = min(safe_time_to_sleep_for, target_frametime.count());
        usleep(clamped_time_to_sleep_for);
        // end of frame time (printing is not included, *though it should be*)
        auto const t2 = std::chrono::steady_clock::now();
        auto const frametime = std::chrono::duration_cast<std::chrono::microseconds>(t2-t0);
        sleep_duration_adjustment    = target_frametime-frametime;
        auto const fps  = 1000000/frametime.count();
        if (fps < 50) {
            // printf("tri_count:  %zu\n", active_buffer.triangle_count);
            // printf("logic_time: %li us\n", logic_time.count());
            // printf("frame_time: %li us\n", frametime.count());
            // printf("adjustment: %li us\n", sleep_duration_adjustment.count());
            // printf("fps:        %li\n", fps);
            // printf("---------------------\n");
        }
        t0 = t2;
    }

    return 0;
}
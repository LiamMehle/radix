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

ExposedRenderData shared_render_data;  // state of the GL thread exposed to other threads

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
    std::array<GLBufferObject, 2> vbos;
    for (auto& vbo : vbos) {
        glGenBuffers(1, &vbo.vbo);
        vbo.vertex_count = 0;
    }

    glBindVertexArray(vao);                                                       // bind configuration object: remembers the global (buffer) state
        // glBindBuffer(GL_ARRAY_BUFFER, vbo);                                           // bind the buffer to the slot for how it will be used
        // glBufferData(GL_ARRAY_BUFFER, sizeof(verticies), verticies, GL_STATIC_DRAW);  // send data to the gpu (with usage hints)
        // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);                        // configure vbo metadata
        // glEnableVertexAttribArray(0);                                                 // enable the config
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
    size_t triangle_count = 0;
    uint_fast8_t current_active_buffer_id = 0;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        glBindVertexArray(vao);

        // GL buffer id of buffer with data being streamed in, in a background context
        uint_fast8_t const current_inactive_buffer_id = current_active_buffer_id ? 0 : 1;
        auto const current_active_buffer = [&]() -> GLBufferObject& { return vbos[current_active_buffer_id]; };
        // bool compare_exchange_weak( T& expected, T desired,
        //                             std::memory_order success,
        //                             std::memory_order failure ) noexcept;
        GLBufferObject* null = nullptr;
        auto const buffer_swap_success = shared_render_data.inactive_buffer.compare_exchange_weak(
            null, &current_active_buffer(),
            std::memory_order_acq_rel,
            std::memory_order_consume);

        if (buffer_swap_success)
            current_active_buffer_id = current_inactive_buffer_id;

        // ----------- drawing -----------
        // clear screen
        glClearColor(.1f, .1f, .1f, 5.f);
        glClear(GL_COLOR_BUFFER_BIT);   
        // pass data to the gpu to be able to perform compute
        glBindVertexArray(vao);
        glUseProgram(program);
        glBindBuffer(GL_ARRAY_BUFFER, current_active_buffer().vbo);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);                        // configure vbo metadata
        glEnableVertexAttribArray(0);                                                 // enable the config
        glDrawArrays(GL_TRIANGLES, 0, current_active_buffer().vertex_count);          // draw call
        glfwSwapBuffers(window);
        // logic time end
        auto const t1 = std::chrono::steady_clock::now();
        auto const logic_time = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0);
        // adjustment is divided by 2 as a heuristic (avoid large +/- swings, effectively P from PID with factor of .5)
        auto const time_to_sleep_for = target_frametime - logic_time + (sleep_duration_adjustment/2);
        // max prevents underflow
        usleep(max(static_cast<int64_t>(0), std::chrono::duration_cast<std::chrono::microseconds>(time_to_sleep_for).count()));
        // end of frame time (printing is not included, *though it should be*)
        auto const t2 = std::chrono::steady_clock::now();
        auto const frametime = std::chrono::duration_cast<std::chrono::microseconds>(t2-t0);
        sleep_duration_adjustment    = target_frametime-frametime;
        printf("vbo handle: %d\n", current_active_buffer().vbo);
        printf("tri_count:  %zu\n", triangle_count);
        printf("logic_time: %li us\n", logic_time.count());
        printf("frame_time: %li us\n", frametime.count());
        printf("adjustment: %li us\n", sleep_duration_adjustment.count());
        printf("fps:        %li\n", 1000000/frametime.count());
        printf("---------------------\n");
        t0 = t2;
    }

    for (auto& vbo : vbos)
        glDeleteBuffers(1, &vbo.vbo);

    return 0;
}
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

ExposedRenderData shared_render_data;  // a set of variables shared with the main GL render thread

void error_callback(int, const char* err_str) {
    printf("GLFW Error: %s\n", err_str);
}

namespace fs = std::filesystem;
std::string binary_path = fs::canonical("/proc/self/exe").parent_path();
// static
// int x_error_handler(Display* display, XErrorEvent* event) {
//     puts("a");
//     return 0;
// }

int main(int argc, char** const argv) {
    ros::init(argc, argv, "radix_node");
    // start ros-related thread
    std::thread ros_thread = std::thread([&]{ros_event_loop(argc, argv);});

    // initialize OpenGL
    glfwSetErrorCallback(error_callback);
    raii::GLFW glfw{};
    if (glfw != GLFW_TRUE) {
        puts("failed to initialize glfw");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    raii::Window window = raii::Window(WIDTH, HEIGHT, "Radix", nullptr, nullptr);
    if (!static_cast<GLFWwindow*>(window)) {
        puts("failed to create a window");
        return 2;
    }

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

    if (!glfwExtensionSupported("GL_ARB_buffer_storage")) {
        puts("[fatal]: No support for GL_ARB_buffer_storage. I couldn't be bothered to write code to deal with this. Needed for buffer mapping");
        return 4;
    }

    // todo-perf: compute shader to expand compressed version of the data
    // buffers:
    raii::VAO vao{};
    {    
        GLuint buffers[2];
        glCreateBuffers(2, buffers);
        shared_render_data.buffers[0].vbo = buffers[0];
        shared_render_data.buffers[1].vbo = buffers[1];
    }
    auto const gl_allocate_trianglebuffer = [](TriangleBuffer& fat_buffer, GLenum const spare_target, size_t const count) {
        glBindBuffer(spare_target, fat_buffer.vbo);
        auto const capacity = count * sizeof(decltype(*fat_buffer.mapping));
        glBufferStorage(spare_target, capacity, nullptr,
            GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT | GL_MAP_WRITE_BIT);
        fat_buffer.capacity = capacity;
    };
    auto const gl_map_buffer = [](GLenum const target, size_t const size) {
        return static_cast<float*>(glMapBufferRange(GL_COPY_WRITE_BUFFER, 0, size,
                    GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT | GL_MAP_WRITE_BIT));
    };

    gl_allocate_trianglebuffer(shared_render_data.buffers[0], GL_COPY_WRITE_BUFFER, 4096);
    gl_allocate_trianglebuffer(shared_render_data.buffers[1], GL_COPY_WRITE_BUFFER, 4096);

    glBindVertexArray(vao);                                                       // bind configuration object: remembers the global (buffer) state
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

    auto const z_uniform = glGetUniformLocation(program, "z");
    print_gl_errors("get z uniform location");
    auto const zoom_uniform = glGetUniformLocation(program, "zoom");
    print_gl_errors("get zoom uniform location");

    // enable vsync if present:
    set_vsync(true);

    using namespace std::chrono_literals;
    auto constexpr target_frametime = (1000000us)/global_base_rate;
    auto t0 = std::chrono::steady_clock::now();
    auto sleep_duration_adjustment = 0us;
    configure_features();
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        auto const   active_buffer_id = [&]                      { return shared_render_data.active_buffer == 0 ? 0 : 1;    };
        auto const inactive_buffer_id = [&]                      { return shared_render_data.active_buffer != 0 ? 0 : 1;    };
        auto const   active_buffer    = [&]() -> TriangleBuffer& { return shared_render_data.buffers[  active_buffer_id()]; };
        auto const inactive_buffer    = [&]() -> TriangleBuffer& { return shared_render_data.buffers[inactive_buffer_id()]; };

        glBindVertexArray(vao);

        // setup transfer to the correct buffer
        // if transfer is done, swap buffers and

        bool const has_ownership = !shared_render_data.copy_in_progress;
        if (has_ownership) {
            bool const copy_was_successful = shared_render_data.needed_capacity <= inactive_buffer().capacity;
            if (copy_was_successful) {
                // cleanup
                // glUnmapBuffer(GL_COPY_WRITE_BUFFER);
                inactive_buffer().mapping = nullptr;
                print_gl_errors("swap-unmap");
                // swap
                shared_render_data.active_buffer = inactive_buffer_id();
                // setup
                glBindBuffer(GL_ARRAY_BUFFER,        active_buffer().vbo);
                print_gl_errors("swap-bind1");
                glBindBuffer(GL_COPY_WRITE_BUFFER, inactive_buffer().vbo);
                print_gl_errors("swap-bind2");
                inactive_buffer().mapping = gl_map_buffer(GL_COPY_WRITE_BUFFER, inactive_buffer().capacity);
                print_gl_errors("swap-map");
            } else {
                glUnmapBuffer(GL_COPY_WRITE_BUFFER);
                inactive_buffer().mapping = nullptr;
                // buffer is immutable, so a new buffer must be created
                print_gl_errors("noswap-unmap");
                glDeleteBuffers(1, &inactive_buffer().vbo);
                inactive_buffer().vbo = 0;
                print_gl_errors("noswap-delete");
                glCreateBuffers(1, &inactive_buffer().vbo);
                print_gl_errors("noswap-create");
                glBindBuffer(GL_COPY_WRITE_BUFFER, inactive_buffer().vbo);
                print_gl_errors("noswap-bind");
                gl_allocate_trianglebuffer(inactive_buffer(), GL_COPY_WRITE_BUFFER, shared_render_data.needed_capacity);
                print_gl_errors("noswap-alloc");
                inactive_buffer().mapping = gl_map_buffer(GL_COPY_WRITE_BUFFER, inactive_buffer().capacity);
                print_gl_errors("noswap-map");
            }
            shared_render_data.copy_in_progress.store(true, std::memory_order_release);
            shared_render_data.update.notify_all();
        }

        // ----------- drawing -----------
        // clear screen
        glClearColor(.1f, .1f, .1f, 5.f);
        print_gl_errors("clear color");
        glClear(GL_COLOR_BUFFER_BIT);   
        print_gl_errors("clear command");


        glBindVertexArray(vao);
        glUseProgram(program);

        glUniform1f(z_uniform, 0.0f);
        glUniform1f(zoom_uniform, active_buffer().zoom);
        print_gl_errors("set zoom uniform");

        glDrawArrays(GL_TRIANGLES, 0, active_buffer().triangle_count);  // draw call
        print_gl_errors("draw triangle array");
        glfwSwapBuffers(window);
        print_gl_errors("swap buffers");
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
            printf("tri_count:  %zu\n", active_buffer().triangle_count);
            printf("logic_time: %li us\n", logic_time.count());
            printf("frame_time: %li us\n", frametime.count());
            printf("adjustment: %li us\n", sleep_duration_adjustment.count());
            printf("fps:        %li\n", fps);
            printf("---------------------\n");
        }
        t0 = t2;
        
        print_gl_errors("uncaught loop errors");
    }

    return 0;
}
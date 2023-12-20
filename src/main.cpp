#undef  __cplusplus
#define __cplusplus 202003L
#include <cstdint>
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
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "gl.h"
#include "utils.hpp"
#include "raii.cpp"

constexpr size_t WIDTH  = 800;
constexpr size_t HEIGHT = 600;

/*
    Base rate at which everything refreshes
    This program is based on polling. This is the base rate, mainly setting the rate a which the display refreshes.
*/
constexpr int global_base_rate = 60;

template<typename T>
static inline constexpr
auto max(T const a, T const b) -> T { return a > b ? a : b; }

void error_callback(int, const char* err_str) {
    printf("GLFW Error: %s\n", err_str);
}

namespace fs = std::filesystem;
std::string binary_path = fs::canonical("/proc/self/exe").parent_path();
std::string read_file(char const* const path) {

    // Open the file using ifstream
    std::ifstream file_stream(path);
    std::stringstream buffer;
    if (file_stream.is_open()) {
        buffer << file_stream.rdbuf();
        file_stream.close();
    } else
        printf("[error]:failed to read file: %s\n", path);
    return buffer.str();
}

raii::Shader compile_shader(std::string const& src, GLenum const type) {
    raii::Shader shader(type);
    GLint success;
    std::vector<GLchar> program_log(1024, 0);
    auto source_ptr = src.c_str();
    auto source_len = src.size();
    glShaderSource(shader, 1, reinterpret_cast<GLchar const**>(&source_ptr), reinterpret_cast<GLint const*>(&source_len));
    glCompileShader(shader);
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, program_log.size(), nullptr, program_log.data());
        printf("[error]: %s\n", program_log.data());
        return 0;
    }
    return shader;
}


int x_error_handler(Display* display, XErrorEvent* event) {
    puts("a");
    return 0;
}

// sets the vsync state to the boolean passed in
// returns true on success
bool set_vsync(bool const enabled) {
    typedef void (*glXSwapIntervalEXT_t)(Display *dpy, GLXDrawable drawable, int interval);
    
    Display* display = glXGetCurrentDisplay();
    if (!display) {
        printf("[error]: failed to open X display\n");
        return false;
    }

    const char *glx_client_extensions = glXGetClientString(display, GLX_EXTENSIONS);
    puts(glx_client_extensions);

    glXSwapIntervalEXT_t glXSwapIntervalEXT = 
        (glXSwapIntervalEXT_t)glXGetProcAddress((const GLubyte*)"glXSwapIntervalEXT");
    if (glXSwapIntervalEXT == nullptr) {
        printf("[error]: glXSwapIntervalEXT not found\n");

        return false;
    }

    GLXDrawable drawable = glXGetCurrentDrawable();

    glXSwapIntervalEXT(display, drawable, enabled ? 1 : 0);

    return true;
}

enum Direction {
    Left, Right
};

void ros_event_loop(int argc, char** const argv, raii::Window const& window) {
    ros::NodeHandle n;
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
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    raii::Window window = raii::Window(WIDTH, HEIGHT, "Radix", nullptr, nullptr);
    if (!static_cast<GLFWwindow*>(window)) {
        puts("failed to create a window");
        return 2;
    }
    // start ros-related thread
    // std::thread ros_thread = std::thread([&]{ros_event_loop(argc, argv, window);});

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

    GLfloat verticies[] = {
        -1., -1., 0.,
         1., -1., 0.,
         0.,  1., 0.
    };

    // buffers:
    raii::VAO vao{};
    raii::VBO vbo{};

    glBindVertexArray(vao);                                                       // bind configuration object: remembers the global (buffer) state
    glBindBuffer(GL_ARRAY_BUFFER, vbo);                                           // bind the buffer to the slot for how it will be used
    glBufferData(GL_ARRAY_BUFFER, sizeof(verticies), verticies, GL_STATIC_DRAW);  // send data to the gpu (with usage hints)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);                        // configure vbo metadata
    glEnableVertexAttribArray(0);                                                 // enable the config

    glBindVertexArray(0);                                                         // unbinding the buffers (safety?)

    // shaders:
    auto const vertex_shader_path   = binary_path + "/vertex.glsl";
    auto const fragment_shader_path = binary_path + "/fragment.glsl";
    std::string vertex_shader_src = read_file(vertex_shader_path.c_str());
    std::string fragment_shader_src = read_file(fragment_shader_path.c_str());

    raii::Program program{};
    raii::Shader vertex_shader   = compile_shader(vertex_shader_src,   GL_VERTEX_SHADER);
    raii::Shader fragment_shader = compile_shader(fragment_shader_src, GL_FRAGMENT_SHADER);
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

    GLint const x_offset_uniform = glGetUniformLocation(program, "x_offset");

    // enable vsync if present:
    set_vsync(true);

    // logic
    Direction movement_direction = Left;
    float triangle_offset = 0;
    float constexpr triangle_max_offset = 0.7;
    float constexpr triangle_increment = 0.005;
    using namespace std::chrono_literals;
    auto constexpr target_frametime = (1000000us)/global_base_rate;
    auto t0 = std::chrono::steady_clock::now();
    auto sleep_duration_adjustment = 0us;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // logic
        if (abs(triangle_offset) + triangle_increment > triangle_max_offset)
            movement_direction = movement_direction == Right ? Left : Right;

        triangle_offset += movement_direction == Right ? triangle_increment : -triangle_increment;

        // ----------- drawing -----------
        // clear screen
        glClearColor(.1f, .1f, .1f, 5.f);
        glClear(GL_COLOR_BUFFER_BIT);       

        // draw call
        glUseProgram(program);
        glBindVertexArray(vao);
        glUniform1f(x_offset_uniform, triangle_offset);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
        glfwSwapBuffers(window);
        glFinish();
        auto const t1 = std::chrono::steady_clock::now();
        auto const time_elapsed_computing = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0);
        auto const time_to_sleep_for = target_frametime - time_elapsed_computing + (sleep_duration_adjustment/2);
        usleep(max(static_cast<int64_t>(0), std::chrono::duration_cast<std::chrono::microseconds>(time_to_sleep_for).count()));
        auto const t2 = std::chrono::steady_clock::now();
        sleep_duration_adjustment    = target_frametime-std::chrono::duration_cast<std::chrono::microseconds>(t2-t0);
        printf("logic_time: %li us\n", std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count());
        printf("frame_time: %li us\n", std::chrono::duration_cast<std::chrono::microseconds>(t2-t0).count());
        printf("adjustment: %li us\n", (sleep_duration_adjustment).count());
        printf("---------------------\n");
        t0 = t2;
    }

    return 0;
}
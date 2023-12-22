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
#include "utils.hpp"
#include "raii.cpp"
#include "global_config.hpp"
#include "ros_event_loop.hpp"

/*
    Base rate at which everything refreshes
    This program is based on polling. This is the base rate, mainly setting the rate a which the display refreshes.
*/

template<typename T>
static inline constexpr
auto max(T const a, T const b) -> T { return a > b ? a : b; }
template<typename T>
static inline constexpr
auto abs(T const a) -> T { return a >= 0 ? a : -a; }

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

static
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

static
int x_error_handler(Display* display, XErrorEvent* event) {
    puts("a");
    return 0;
}

// sets the vsync state to the boolean passed in
// returns true on success
static
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
    raii::VBO vbo{};

    glBindVertexArray(vao);                                                       // bind configuration object: remembers the global (buffer) state
    glBindBuffer(GL_ARRAY_BUFFER, vbo);                                           // bind the buffer to the slot for how it will be used
    // glBufferData(GL_ARRAY_BUFFER, sizeof(verticies), verticies, GL_STATIC_DRAW);  // send data to the gpu (with usage hints)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 0, 0);                         // configure vbo metadata
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

    GLint const x_max_uniform_handle = glGetUniformLocation(program, "x_max");
    GLint const y_max_uniform_handle = glGetUniformLocation(program, "x_max");

    // enable vsync if present:
    set_vsync(true);

    using namespace std::chrono_literals;
    auto constexpr target_frametime = (1000000us)/global_base_rate;
    auto t0 = std::chrono::steady_clock::now();
    auto sleep_duration_adjustment = 0us;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // ----------- drawing -----------
        // clear screen
        glClearColor(.1f, .1f, .1f, 5.f);
        glClear(GL_COLOR_BUFFER_BIT);       

        // draw call
        glUseProgram(program);
        glBindVertexArray(vao);
        // {
            // std::lock_guard point_cloud_guard(point_cloud_mutex);
            point_cloud_mutex.lock();
            std::vector<float> point_cloud_triangles(point_cloud_points.size()*3, 0.f);
            float* const point_cloud_triangle_ptr = point_cloud_triangles.data();
            float* const point_cloud_points_ptr = point_cloud_points.data();
            float x_max = 0;
            float y_max = 0;
            for (size_t point_index=0; point_index<point_cloud_points.size(); point_index+=3) {
                // triangle has 3x the coordinates of a point, thus:
                size_t const triangle_index    = 3*point_index;
                float* const point_ptr         = point_cloud_points_ptr+point_index;
                float* const triangle_vertex_0 = point_cloud_triangle_ptr+triangle_index;
                float* const triangle_vertex_1 = triangle_vertex_0 + 3;
                float* const triangle_vertex_2 = triangle_vertex_1 + 3;
                auto const x = point_ptr[0];
                auto const y = point_ptr[1];
                auto const z = point_ptr[2];
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
            point_cloud_mutex.unlock();
            // for (size_t i=0; i<point_cloud_triangles.size(); i+=3) {
            //     auto vertex_ptr = point_cloud_triangle_ptr + i;
            //     vertex_ptr[0] *= 1/x_max;
            //     vertex_ptr[1] *= 1/y_max;
            // }
            glBufferData(GL_ARRAY_BUFFER, point_cloud_triangles.size()*sizeof(point_cloud_triangles[0]), point_cloud_triangles.data(), GL_STREAM_DRAW);
            printf("x_max: %f\n", x_max);
            printf("y_max: %f\n", y_max);
            glFinish();
        // }
        glUniform1f(x_max_uniform_handle, x_max_uniform_handle);
        glUniform1f(y_max_uniform_handle, y_max_uniform_handle);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
        glfwSwapBuffers(window);
        glFinish();
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
        printf("logic_time: %li us\n", logic_time.count());
        printf("frame_time: %li us\n", frametime.count());
        printf("adjustment: %li us\n", sleep_duration_adjustment.count());
        printf("---------------------\n");
        t0 = t2;
    }

    return 0;
}
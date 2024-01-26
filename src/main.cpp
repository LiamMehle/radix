// main logic of the program
// #define DEBUG_OUTPUT
#include <cstdio>
#include <cstdint>
#include <cassert>
#include <cstring>
#include <string>
#include <vector>
#include <thread>
#include <filesystem>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "gl.h"
#include "gl_tools.hpp"
#include "utils.hpp"
#include "raii.cpp"
#include "functional.hpp"
#include "global_config.hpp"
#include "ros_event_loop.hpp"
#include "text.hpp"

static
void error_callback(int, const char* err_str) {
   std::printf("GLFW Error: %s\n", err_str);
}

namespace fs = std::filesystem;
std::string binary_path = fs::canonical("/proc/self/exe").parent_path();
static
int x_error_handler(Display* display, XErrorEvent* event) {
    std::puts("a");
    return 0;
}

// data used for rendering of a frame
enum PrivateRenderDataFlagBits {
    perimeter_enabled = 1,
};
struct DrawCallInfo {
    GLenum draw_mode;
    GLuint vao;
    GLuint vertex_offset;
    GLuint vertex_count;
};
struct PrivateRenderData {
    uint32_t flags;
    DrawCallInfo point_cloud_draw_info;
    DrawCallInfo perimeter_draw_info;
};
struct SizedVbo {
    GLuint vbo;
    GLint vertex_count;
};
struct TextRenderResource {
    FullProgram program;
    GLuint vao;
    GLuint texture;
    GLuint vertex_buffer_object;
    GLuint sampler;
    GLint top_uniform_location;
    GLint left_uniform_location;
    GLint bottom_uniform_location;
    GLint right_uniform_location;
    GLint color_uniform_location;
    GLint sampler_uniform_location;
};

static
PrivateRenderData private_render_data;
ExposedRenderData shared_render_data;  // state of the GL thread exposed to other threads

static
void draw_entity(DrawCallInfo const draw_info) {
    glBindVertexArray(draw_info.vao);
    glDrawArrays(draw_info.draw_mode, draw_info.vertex_offset, draw_info.vertex_count);
}

// window redraw callback, do not call directly, use draw_point_cloud instead
// assumes private render data contains all relevant correct data and *only draws (based on) said data to the screen*
static
void draw_window(GLFWwindow* window) {
    glfwMakeContextCurrent(window);
    // clear screen
    // glClearColor(.1f, .1f, .1f, 5.f);
    // glClear(GL_COLOR_BUFFER_BIT);

    draw_entity(private_render_data.point_cloud_draw_info);

    // draw perimeter (if enabled)
    if (private_render_data.flags & PrivateRenderDataFlagBits::perimeter_enabled)
        draw_entity(private_render_data.perimeter_draw_info);

    glfwSwapBuffers(window);
}

template <typename T>
static
GLuint convert_to_vbo(GLFWwindow* window, std::vector<T> const& verticies, GLenum const target) {
    GLint const vertex_count = verticies.size();
    GLint const required_buffer_size = sizeof(T) * vertex_count;
    glBufferData(target, required_buffer_size, verticies.data(), GL_STATIC_DRAW);
    return vertex_count;
}


int main(int argc, char** const argv) {
    FT_Library library;
    if (FT_Init_FreeType(&library)) {
        std::puts("failed to initialize freetype");
        return 1;
    }
    FT_Face face;
    if (FT_New_Face(library, "/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf", 0, &face )) {
        std::puts("failed to acquire faccia");
        return 2;
    }


    ros::init(argc, argv, "radix_node");
    // initialize OpenGL
    int status = 0;
    glfwSetErrorCallback(error_callback);
    raii::GLFW glfw{};
    if (glfw != GLFW_TRUE) {
        std::puts("failed to initialize glfw");
        return 3;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    raii::Window window = raii::Window(WIDTH, HEIGHT, "Radix", nullptr, nullptr);
    if (!static_cast<GLFWwindow*>(window)) {
        std::puts("failed to create a window");
        return 4;
    }
    glfwSetInputMode(window, GLFW_STICKY_MOUSE_BUTTONS, GLFW_TRUE);

    // start ros-related thread
    std::thread ros_thread = std::thread([&]{ros_event_loop(argc, argv, window);});

    // continue configuration of window/context
    int frame_buffer_width, frame_buffer_height;
    glfwGetFramebufferSize(window, &frame_buffer_width, &frame_buffer_height);

    glfwMakeContextCurrent(window);
#ifndef NODEBUG
    std::printf("vendor:   %s\n", reinterpret_cast<char const*>(glGetString(GL_VENDOR)));
    std::printf("renderer: %s\n", reinterpret_cast<char const*>(glGetString(GL_RENDERER)));
#endif

    glewExperimental = GL_TRUE;

    if (glewInit() != GLEW_OK) {
        std::puts("failed to init glew");
        return 5;
    }

    glViewport(0, 0, frame_buffer_width, frame_buffer_height);

    // configure the perimeter render data because it is handled asynchronously
    raii::VAO perimeter_vao;
    private_render_data.perimeter_draw_info.vao = perimeter_vao;
    raii::VBO perimeter_vbo;

    raii::VAO point_cloud_vao{};
    std::array<GLBufferObject, 2> vbos;
    for (auto& vbo : vbos) {
        glGenBuffers(1, &vbo.vbo);
        vbo.vertex_count = 0;
    }

    // shaders:
    auto const point_cloud_vertex_shader_path   = binary_path + "/point_cloud_vertex.glsl";
    auto const point_cloud_fragment_shader_path = binary_path + "/point_cloud_fragment.glsl";
    auto const perimeter_vertex_shader_path     = binary_path + "/perimeter_vertex.glsl";
    auto const perimeter_fragment_shader_path   = binary_path + "/perimeter_fragment.glsl";
    auto const text_vertex_shader_path   = binary_path + "/text_vertex.glsl";
    auto const text_fragment_shader_path   = binary_path + "/text_fragment.glsl";
    FullProgram const point_cloud_program = create_program_from_path(point_cloud_vertex_shader_path.c_str(), point_cloud_fragment_shader_path.c_str());
    FullProgram const perimeter_program   = create_program_from_path(perimeter_vertex_shader_path.c_str(), perimeter_fragment_shader_path.c_str());
    FullProgram const text_program        = create_program_from_path(text_vertex_shader_path.c_str(), text_fragment_shader_path.c_str());
    // enable vsync if present:
    // set_vsync(true);

    using namespace std::chrono_literals;
    auto constexpr target_frametime = (1000000us)/global_base_rate;
    auto t0 = std::chrono::steady_clock::now();
    auto sleep_duration_adjustment = 0us;
    bool data_is_loaded = false;
    std::size_t triangle_count = 0;
    uint_fast8_t current_active_buffer_id = 0;
    std::vector<struct CursorPosition> click_points{};
    glfwSetWindowRefreshCallback(window, draw_window);
    // glfwSetMouseButtonCallback(window, handle_mouse_press);
    bool left_mouse_was_pressed = false;

    // set up text rendering resource
    GLuint sampler;
    glGenSamplers(1, &sampler);

    TextRenderResource text_rendering_resource = {
        .program = text_program,
        .vao = 0,
        .texture = 0,
        .vertex_buffer_object = 0,
        .sampler = sampler,
        .top_uniform_location    = glGetUniformLocation(text_program.program, "top"),
        .left_uniform_location   = glGetUniformLocation(text_program.program, "left"),
        .bottom_uniform_location = glGetUniformLocation(text_program.program, "bottom"),
        .right_uniform_location  = glGetUniformLocation(text_program.program, "right"),
        .color_uniform_location  = glGetUniformLocation(text_program.program, "color"),
        .sampler_uniform_location = glGetUniformLocation(text_program.program, "text_bitmap"),
    };
    glGenVertexArrays(1, &text_rendering_resource.vao);
    glGenTextures(1, &text_rendering_resource.texture);
    glGenBuffers(1, &text_rendering_resource.vertex_buffer_object);
    // done setting up text rendering resource

    auto const render_character_bitmap = [text_rendering_resource](FT_Bitmap const* const bitmap, auto const left, auto const top, auto const right, auto const bottom) {
        // freetype suggestions:
        // - linear blending
        // - bitmap is applied to alpha
        auto const r = text_rendering_resource;
        glBindVertexArray(r.vao);

        glBindBuffer(GL_VERTEX_ARRAY, r.vertex_buffer_object);
        float billboard[] = {
            left, top,
            left, bottom,
            right, top,
            right, top,
            left, bottom,
            right, bottom
        };
        glBufferData(GL_VERTEX_ARRAY, 6*2, &billboard, GL_STREAM_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);

        glBindSampler(0, r.sampler);
        glActiveTexture(GL_TEXTURE0 + 0);
        glBindTexture(GL_TEXTURE_2D, r.texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, bitmap->width, bitmap->rows, 0, GL_RED, GL_UNSIGNED_BYTE, bitmap->buffer);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glUniform1f(r.top_uniform_location,    top);
        glUniform1f(r.left_uniform_location,   left);
        glUniform1f(r.bottom_uniform_location, bottom);
        glUniform1f(r.right_uniform_location,  right);
        glUniform3f(r.color_uniform_location,  1.f, 1.f, 1.f);
        glEnable(GL_BLEND);
        glDrawArrays(GL_TRIANGLES, 0, 6*2);
        glBindVertexArray(0);
    };

    while (!glfwWindowShouldClose(window)) {
        bool should_redraw = false;
        glfwPollEvents();
        bool const left_mouse_is_pressed = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        if (left_mouse_is_pressed && (!left_mouse_was_pressed)) {
            should_redraw |= true;
            // update perimeter display
            // todo: wait for last transfer to finish before updating
            // clickpoints due to iterator invalidation

            // setup draw info
            auto cursor_pos = get_cursor_pos(window);
            click_points.push_back(cursor_pos);
            auto& draw_info = private_render_data.perimeter_draw_info;
            draw_info.vao = perimeter_vao;
            glBindVertexArray(draw_info.vao);
            configure_features();
            glUseProgram(perimeter_program.program);
            glBindBuffer(GL_ARRAY_BUFFER, perimeter_vbo);
            GLuint const vertex_count = click_points.size();
            GLuint const required_buffer_size = sizeof(click_points[0]) * vertex_count;
            glBufferData(GL_ARRAY_BUFFER, required_buffer_size, click_points.data(), GL_DYNAMIC_DRAW);

            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
            glEnableVertexAttribArray(0);

            auto const vertex_count_for_drawing = vertex_count;
            draw_info = {
                .draw_mode = GL_LINE_LOOP,
                .vao = draw_info.vao,
                .vertex_offset = 0,
                .vertex_count = vertex_count_for_drawing,
            };
            private_render_data.flags = (vertex_count_for_drawing != 0) * PrivateRenderDataFlagBits::perimeter_enabled
                                      | (private_render_data.flags     & ~PrivateRenderDataFlagBits::perimeter_enabled);
            glBindVertexArray(0);
        }
        left_mouse_was_pressed = left_mouse_is_pressed;

        // GL buffer id of buffer with data being streamed in, in a background context
        uint_fast8_t const current_inactive_buffer_id = current_active_buffer_id ? 0 : 1;
        auto const current_active_buffer = [&]() -> GLBufferObject& { return vbos[current_active_buffer_id]; };

        GLBufferObject* null = nullptr;
        auto const buffer_swap_success = shared_render_data.inactive_buffer.compare_exchange_weak(
            null, &current_active_buffer(),
            std::memory_order_acq_rel,
            std::memory_order_consume);

        if (buffer_swap_success) {
            should_redraw |= true;
            current_active_buffer_id = current_inactive_buffer_id;

            // ----------- drawing -----------

            glBindVertexArray(point_cloud_vao);
            configure_features();
            glUseProgram(point_cloud_program.program);
            glBindBuffer(GL_VERTEX_ARRAY, current_active_buffer().vbo);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);  // configure point_cloud_vbo metadata
            glEnableVertexAttribArray(0);                           // enable the config
            DrawCallInfo draw_info {
                .draw_mode = GL_TRIANGLES,
                .vao = point_cloud_vao,
                .vertex_offset = 0,
                .vertex_count = static_cast<GLuint>(current_active_buffer().vertex_count),
            };
            glBindVertexArray(0);
        }

        // temp rendering code
        glClearColor(.1f, .1f, .1f, 5.f);
        glClear(GL_COLOR_BUFFER_BIT);

        // test drawing text
        char text[] = "hi";
        render_char(
            face,
            text,
            sizeof(text)-1,
            -1.f,
            -1.f,
            1.f/64.f,
            render_character_bitmap
        );

        if (should_redraw)
            draw_window(window);
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
        sleep_duration_adjustment = target_frametime-frametime;
#ifdef DEBUG_OUTPUT
#ifdef FPS
        std::printf("vbo handle: %d\n", current_active_buffer().vbo);
        std::printf("tri_count:  %zu\n", triangle_count);
        std::printf("logic_time: %li us\n", logic_time.count());
        std::printf("frame_time: %li us\n", frametime.count());
        std::printf("adjustment: %li us\n", sleep_duration_adjustment.count());
        std::printf("fps:        %li\n", 1000000/frametime.count());
        std::printf("---------------------\n");
#endif
#endif
        t0 = t2;
    }

    for (auto& vbo : vbos)
        glDeleteBuffers(1, &vbo.vbo);

    return 0;
}

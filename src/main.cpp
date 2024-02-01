// main logic of the program
// #define DEBUG_OUTPUT
// #define DEBUG_FPS
#include <cstdio>
#include <string>
#include <vector>
#include <thread>
#include <filesystem>
#include <ros/ros.h>
#include "gl.h"
#include "gl_tools.hpp"
#include "functional.hpp"
#include "global_config.hpp"
#include "ros_event_loop.hpp"
#include "text.hpp"

static
size_t constexpr UNIFORM_COUNT = 4;
static
size_t constexpr TEXTURE_UNIT_COUNT = 4;
#include "types.hpp"


static
PersistentRenderState refresh_render_state(PersistentRenderState state);

static
void error_callback(int, const char* err_str) {
   std::printf("GLFW Error: %s\n", err_str);
}

namespace fs = std::filesystem;
std::string binary_path = fs::canonical("/proc/self/exe").parent_path();

static
PrivateRenderData private_render_data;
ExposedRenderData shared_render_data = { nullptr };  // state of the GL thread exposed to other threads

static
void draw_entity(DrawCallInfo const draw_info) {
    // bind Vertex data
    glBindVertexArray(draw_info.vao);

    // configure uniforms
     for (int i=0; i<draw_info.uniform_count; i++) {
         switch (draw_info.uniforms[i].type) {
             case UniformConfig::UniformType::Integer1:
                 glUniform1i(draw_info.uniforms[i].location, draw_info.uniforms[i].value.integer);
         }
     }
    // configure textures
    for (int i=0; i<draw_info.texture_unit_count; i++) {
        glActiveTexture(GL_TEXTURE0+i);  // selects the active texture UNIT, not texture itself
        glBindTexture(GL_TEXTURE_2D, draw_info.texture_units[i].assigned_texture);
    }
    // draw
    glDrawArrays(draw_info.draw_mode, draw_info.vertex_offset, draw_info.vertex_count);
    glBindVertexArray(0);
}

// window redraw callback, do not call directly, use draw_point_cloud instead
// assumes private render data contains all relevant correct data and *only draws (based on) said data to the screen*
static
void draw_window(GLFWwindow* window) {
    glfwMakeContextCurrent(window);
    // clear screen
    glClearColor(.1f, .1f, .1f, 5.f);
    glClear(GL_COLOR_BUFFER_BIT);

    for (auto const& maybe_drawable : private_render_data.drawables)
        if (maybe_drawable.has_value())
            draw_entity(maybe_drawable.value());

    glfwSwapBuffers(window);
}

int main(int argc, char** const argv) {
    try {
    ros::init(argc, argv, "radix_node");
    // initialize OpenGL
    glfwSetErrorCallback(error_callback);
    auto const glfw = glfwInit();
    if (glfw != GLFW_TRUE) {
        std::puts("failed to initialize glfw");
        return 3;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    auto const window = glfwCreateWindow(WIDTH, HEIGHT, "Radix", nullptr, nullptr);
    if (!window) {
        std::puts("failed to create a window");
        return 4;
    }
    glfwSetInputMode(window, GLFW_STICKY_MOUSE_BUTTONS, GLFW_TRUE);

    // start ros-related thread

    // continue configuration of window/context
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

    std::thread ros_thread = std::thread([&]{ros_event_loop(argc, argv, window);});

    // configure the perimeter render data because it is handled asynchronously
    GLuint point_cloud_vao, perimeter_vao, perimeter_vbo;
    glGenVertexArrays(1, &perimeter_vao);
    glGenBuffers(1, &perimeter_vbo);

    glGenVertexArrays(1, &point_cloud_vao);
    std::array<GLBufferObject, 2> vbos {0};
    for (auto& vbo : vbos) {
        glGenBuffers(1, &vbo.vbo);
        vbo.vertex_count = 0;
    }

    // shaders:
    auto const point_cloud_vertex_shader_path   = binary_path + "/point_cloud_vertex.glsl";
    auto const point_cloud_fragment_shader_path = binary_path + "/point_cloud_fragment.glsl";
    auto const perimeter_vertex_shader_path     = binary_path + "/perimeter_vertex.glsl";
    auto const perimeter_fragment_shader_path   = binary_path + "/perimeter_fragment.glsl";
    auto const text_vertex_shader_path          = binary_path + "/text_vertex.glsl";
    auto const text_fragment_shader_path        = binary_path + "/text_fragment.glsl";
    FullProgram const point_cloud_program = create_program_from_path(point_cloud_vertex_shader_path.c_str(), point_cloud_fragment_shader_path.c_str());
    FullProgram const perimeter_program   = create_program_from_path(perimeter_vertex_shader_path.c_str(), perimeter_fragment_shader_path.c_str());
    FullProgram const text_program        = create_program_from_path(text_vertex_shader_path.c_str(), text_fragment_shader_path.c_str());
    // enable vsync if present:
    // set_vsync(true);

    FT_Library library;
    if (FT_Init_FreeType(&library)) {
        std::puts("failed to initialize freetype");
        return 1;
    }
    FT_Face face;
    if (FT_New_Face(library, "/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf", 0, &face )) {
        std::puts("failed to acquire typeface");
        return 2;
    }
//    FT_Set_Char_Size(face,0,72 * 64 + 32, 300, 300);
    FT_Set_Pixel_Sizes(face, 0, 48);
    auto const big_charset = load_charset<0, 128>(face);
//    FT_Set_Char_Size(face,0,12 * 64 + 32, 300, 300);
    FT_Set_Pixel_Sizes(face, 0, 48);
    auto const small_charset = load_charset<0, 128>(face);
    FT_Done_FreeType(library);

    using namespace std::chrono_literals;
    auto constexpr target_frametime = (1000000us)/global_base_rate;
    configure_features();
    TextRenderResource text_rendering_resource = {
        .program = text_program,
        .sampler_uniform_location = glGetUniformLocation(text_program.program, "text_bitmap"),
        .pitch_uniform_location = glGetUniformLocation(text_program.program, "pitch"),
        .big_charset = big_charset,
        .small_charset = small_charset,
    };
    glGenVertexArrays(1, &text_rendering_resource.vao);

    

    PersistentRenderState render_state {
        .sleep_duration_adjustment = 0us,
        .target_frametime = target_frametime,
        .perimeter_vao = perimeter_vao,
        .perimeter_vbo = perimeter_vbo,
        .point_cloud_vao = point_cloud_vao,
        .point_cloud_program = point_cloud_program,
        .perimeter_program = perimeter_program,
        .window = window,
        .click_points = {},
        .objects_for_cleanup = {},
        .left_mouse_was_pressed = false,
        .vbos = vbos,
        .t0 = std::chrono::steady_clock::now(),
        .current_active_buffer_id = 0,
        .text_rendering_resource = text_rendering_resource,
    };
    glfwSetWindowRefreshCallback(window, draw_window);
    while (!glfwWindowShouldClose(window))
        render_state = refresh_render_state(std::move(render_state));

    for (auto& vbo : vbos)
        glDeleteBuffers(1, &vbo.vbo);

    glfwTerminate();
    return 0;
    } catch (std::exception&) {
        // ???
        return -1;
    }
}

static
PersistentRenderState refresh_render_state(PersistentRenderState state) {
    private_render_data.drawables.clear();
    // bool should_redraw = false;
    glfwPollEvents();
    // bool const left_mouse_is_pressed = glfwGetMouseButton(state.window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    // if (left_mouse_is_pressed && (!state.left_mouse_was_pressed)) {
    //     should_redraw |= true;
    //     // update perimeter display
    //     // todo: wait for last transfer to finish before updating
    //     // clickpoints due to iterator invalidation

    //     // setup draw info
    //     auto cursor_pos = get_cursor_pos(state.window);
    //     state.click_points.push_back(cursor_pos);
    //     DrawCallInfo draw_info = {0};// = private_render_data.perimeter_draw_info;
    //     draw_info.vao = state.perimeter_vao;
    //     glBindVertexArray(draw_info.vao);
    //     glUseProgram(state.perimeter_program.program);
    //     glBindBuffer(GL_ARRAY_BUFFER, state.perimeter_vbo);
    //     GLuint const vertex_count = state.click_points.size();
    //     GLuint const required_buffer_size = sizeof(state.click_points[0]) * vertex_count;
    //     glBufferData(GL_ARRAY_BUFFER, required_buffer_size, state.click_points.data(), GL_STATIC_DRAW);

    //     glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
    //     glEnableVertexAttribArray(0);

    //     auto const vertex_count_for_drawing = vertex_count;
    //     draw_info = {
    //         .vao = draw_info.vao,
    //         .draw_mode = GL_LINE_LOOP,
    //         .vertex_offset = 0,
    //         .vertex_count = vertex_count_for_drawing,
    //     };
    //     private_render_data.drawables.push_back(draw_info);
    //     glBindVertexArray(0);
    // }
    // state.left_mouse_was_pressed = left_mouse_is_pressed;

    // // GL buffer id of buffer with data being streamed in, in a background context
    // uint_fast8_t const current_inactive_buffer_id = state.current_active_buffer_id ? 0 : 1;
    // auto const current_active_buffer = [&]() -> GLBufferObject& { return state.vbos[state.current_active_buffer_id]; };

    // GLBufferObject* null = nullptr;
    // auto const buffer_swap_success = shared_render_data.inactive_buffer.compare_exchange_weak(
    //     null, &current_active_buffer(),
    //     std::memory_order_acq_rel,
    //     std::memory_order_consume);

    //     // text rendering
    //     for (auto const& entity : state.objects_for_cleanup) {
    //         glDeleteVertexArrays(1, &entity.vao);
    //         glDeleteBuffers(1, &entity.vbo);
    //     }
    //     state.objects_for_cleanup.clear();

    auto const render_character_bitmap = [text_rendering_resource = state.text_rendering_resource](
        Bitmap const& bitmap,
        auto const left,
        auto const top,
        auto const right,
        auto const bottom) -> std::pair<DrawCallInfo, DrawCallCleanupInfo> {
        // freetype suggestions:
        // - linear blending
        // - bitmap is applied to alpha
        auto const r = text_rendering_resource;
        GLuint vao, vbo;

        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);

        glUseProgram(r.program.program);
        float billboard[] = {
            left,  top,    0.f, 1.f,
            left,  bottom, 0.f, 0.f,
            right, top,    1.f, 1.f,
            right, top,    1.f, 1.f,
            right, bottom, 1.f, 0.f,
            left,  bottom, 0.f, 0.f,
        };
        glBufferData(GL_ARRAY_BUFFER, sizeof(billboard), &billboard, GL_STREAM_DRAW);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);

#ifdef DEBUG_FT
        {
            auto buffer = static_cast<uint8_t*>(alloca(bitmap.width*bitmap.height));
            glReadPixels(0, 0, bitmap.width, bitmap.height, GL_RED, GL_UNSIGNED_BYTE, buffer);
            for (int i=0; i<bitmap.height; i++) {
                for (int j=0; j<bitmap.width; j++) {
                    putchar(buffer[i*bitmap.pitch+j] >= 1 ? '#' : ' ');
                }
                puts("");
            }
            puts("'''''''''''''");
        }
#endif

        // glActiveTexture(GL_TEXTURE0);  // selects the active texture UNIT, not texture itself
        // glBindTexture(GL_TEXTURE_2D, bitmap.texture);  // a texture is bound to the texture UNIT
        // debug_invoke(glBindSampler, 0, r.sampler);  // a sampler id is bound to the sampler. A sampler is a mechanism that fetches the underlaying data and transforms is in some specified way
        // glDisable(GL_COLOR_LOGIC_OP);
        // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // glEnable(GL_BLEND);
        // glUniform1i(r.sampler_uniform_location, 0);

        glBindVertexArray(0);

        return std::pair<DrawCallInfo, DrawCallCleanupInfo> {
            DrawCallInfo {
                .vao = vao,
                .draw_mode = GL_TRIANGLES,
                .vertex_offset = 0,
                .vertex_count = 6,
                .uniform_count = 2,
                .uniforms = {
                    UniformConfig{
                        .type = UniformConfig::UniformType::Integer1,
                        .location = r.sampler_uniform_location,
                        .value = {
                                .integer = 0,
                        },
                    },
                    UniformConfig{
                        .type = UniformConfig::UniformType::Integer1,
                        .location = r.pitch_uniform_location,
                        .value = {
                                .integer = bitmap.pitch,
                        },
                    },
                },
                .texture_unit_count = 1,
                .texture_units = {
                    bitmap.texture,
                },
            },
            DrawCallCleanupInfo {
                .vao = vao,
                .vbo = vbo
            }
        };
    };
    char constexpr text[] = "hello";
    auto top = 0.f;
    auto left = 0.f;
    int window_width, window_height;
    glfwGetWindowSize(state.window, &window_width, &window_height);

    auto const pixel_size = 2.f/static_cast<float>(window_height);
    for (int i=0; i<sizeof(text)-1; i++) {
        auto const c = text[i];
        auto const optional_bitmap = state.text_rendering_resource.small_charset.bitmap[c];
        if (optional_bitmap.has_value()) {
            auto const bitmap = optional_bitmap.value();
            auto const width =  static_cast<float>(bitmap.width) * pixel_size;
            auto const height = static_cast<float>(bitmap.height) * pixel_size;
            auto [render_info, cleanup_info] = render_character_bitmap(bitmap, left, top, left+width, top+height);
            private_render_data.drawables.emplace_back(render_info);
            state.objects_for_cleanup.emplace_back(cleanup_info);
            left += width;
        }
    }
    for (auto const& entity : private_render_data.drawables)
        if (entity.has_value())
            draw_entity(entity.value());
    glfwSwapBuffers(state.window);

    // if (buffer_swap_success) {
    //     should_redraw |= true;
    //     state.current_active_buffer_id = current_inactive_buffer_id;

    //     // ----------- drawing -----------
    //     glBindVertexArray(state.point_cloud_vao);
    //     glUseProgram(state.point_cloud_program.program);
    //     glBindBuffer(GL_ARRAY_BUFFER, current_active_buffer().vbo);
    //     glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);  // configure point_cloud_vbo metadata
    //     glEnableVertexAttribArray(0);                           // enable the config
    //     private_render_data.drawables.emplace_back(DrawCallInfo {
    //         .draw_mode = GL_TRIANGLES,
    //         .vao = state.point_cloud_vao,
    //         .vertex_offset = 0,
    //         .vertex_count = static_cast<GLuint>(current_active_buffer().vertex_count),
    //     });
    //     glBindVertexArray(0);
    // }

    // if (should_redraw)
    //     draw_window(state.window);

    // // logic time end
    // auto const t1 = std::chrono::steady_clock::now();
    // auto const logic_time = std::chrono::duration_cast<std::chrono::microseconds>(t1-state.t0);
    // // adjustment is divided by 2 as a heuristic (avoid large +/- swings, effectively P from PID with factor of .5)
    // auto const time_to_sleep_for = state.target_frametime - logic_time + (state.sleep_duration_adjustment/2);
    // // max prevents underflow
    // usleep(max(static_cast<int64_t>(0), std::chrono::duration_cast<std::chrono::microseconds>(time_to_sleep_for).count()));
    // // end of frame time (printing is not included, *though it should be*)
    // auto const t2 = std::chrono::steady_clock::now();
    // auto const frametime = std::chrono::duration_cast<std::chrono::microseconds>(t2-state.t0);
    // state.sleep_duration_adjustment = state.target_frametime-frametime;
#ifdef DEBUG_FPS
    printf("vbo handle: %d\n", current_active_buffer().vbo);
    printf("tri_count:  %zu\n", triangle_count);
    printf("logic_time: %li us\n", logic_time.count());
    printf("frame_time: %li us\n", frametime.count());
    printf("adjustment: %li us\n", sleep_duration_adjustment.count());
    printf("fps:        %li\n", 1000000/frametime.count());
    printf("---------------------\n");
#endif
    // state.t0 = t2;
    return state;
}
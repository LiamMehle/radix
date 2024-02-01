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
    // GLint top_uniform_location;
    // GLint left_uniform_location;
    // GLint bottom_uniform_location;
    // GLint right_uniform_location;
    // GLint color_uniform_location;
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
    FT_Set_Char_Size(face, 11*64, 11*64, 300,300);

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

    {
        int frame_buffer_width, frame_buffer_height;
        glfwGetFramebufferSize(window, &frame_buffer_width, &frame_buffer_height);
        glViewport(0, 0, frame_buffer_width, frame_buffer_height);
    }

    glfwMakeContextCurrent(window);

    glewExperimental = GL_TRUE;

    if (glewInit() != GLEW_OK) {
        std::puts("failed to init glew");
        return 5;
    }

    // setup:
    //  shaders:
    auto const text_vertex_shader_path          = binary_path + "/text_vertex.glsl";
    auto const text_fragment_shader_path        = binary_path + "/text_fragment.glsl";
    FullProgram const text_program        = create_program_from_path(text_vertex_shader_path.c_str(), text_fragment_shader_path.c_str());
    
    //  textures:
    FT_Load_Char(face, c, FT_LOAD_RENDER)

    //      texture setup:
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, face->glyph->bitmap.width, face->glyph->bitmap.rows, 0, GL_RED, GL_UNSIGNED_BYTE, face->glyph->bitmap.buffer);
    glGenerateMipmap(GL_TEXTURE_2D);

    FT_Done_Face(face);
    FT_Done_FreeType(ft);

    //      texture unit setup:
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        
        glfwSwapBuffers(window)
    }

    return 0;
}

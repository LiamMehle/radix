#pragma once

#include "gl.h"
#include "utils.hpp"
#include "raii.cpp"

static inline
void configure_features() {
    glDisable(GL_BLEND);
    glEnable(GL_COLOR_LOGIC_OP);
    glLogicOp(GL_COPY);
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEBUG_OUTPUT);
    glDisable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDisable(GL_DEPTH_CLAMP);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_DITHER);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_MULTISAMPLE);
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    glDisable(GL_SAMPLE_ALPHA_TO_ONE);
    glDisable(GL_SAMPLE_COVERAGE);
    glEnable(GL_SAMPLE_SHADING);
    glMinSampleShading(.001);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_STENCIL_TEST);
}

// sets the vsync state to the boolean passed in
// returns true on success
static
inline
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
raii::Shader shader_from_file(char const* filename, GLenum const type) {
    auto const source = read_file(filename);
    return compile_shader(source, type);
}
#pragma once

#include <fstream>
#include "gl.h"
#include "utils.hpp"

struct FullProgram {
    GLint program;
    GLint vertex_shader;
    GLint fragment_shader;
};

#ifdef DEBUG_OUTPUT
#include <cstdio>
static inline
GLenum print_gl_errors(char const* const where) {
    GLenum error = glGetError();
    while (error != GL_NO_ERROR) {
        printf("%s:\t%s\n", where, gluErrorString(error));
        error = glGetError();
    }
    return error;
}
#else
static inline
GLenum print_gl_errors(char const* const where){ return GL_NO_ERROR; }
#endif

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

static inline
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

static inline
GLint compile_shader(std::string const& src, GLenum const type) {
    GLint shader = glCreateShader(type);
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

static inline
GLint shader_from_file(char const* filename, GLenum const type) {
    auto const source = read_file(filename);
    return compile_shader(source, type);
}

static inline
GLint create_program(char const* const vertex_shader_path, char const* const fragment_shader_path) {
    // shaders:
    
    GLuint program = glCreateProgram();
    GLint vertex_shader   = shader_from_file(vertex_shader_path,   GL_VERTEX_SHADER);
    GLint fragment_shader = shader_from_file(fragment_shader_path, GL_FRAGMENT_SHADER);
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
    return program;
}

static inline
FullProgram create_program_from_path(char const* const vertex_shader_src_path, char const* const fragment_shader_src_path) {
    GLint program         = glCreateProgram();
    GLint fragment_shader = shader_from_file(fragment_shader_src_path, GL_FRAGMENT_SHADER);
    GLint vertex_shader   = shader_from_file(vertex_shader_src_path,   GL_VERTEX_SHADER);

    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    GLint result;
    glGetProgramiv(program, GL_LINK_STATUS, &result);
    if (!result) {
        std::vector<GLchar> program_log(1024, 0);
        glGetProgramInfoLog(program, program_log.size(), nullptr, program_log.data());
        printf("[error]: %s\n", program_log.data());
        return {0};
    }
    glValidateProgram(program);
    glGetProgramiv(program, GL_VALIDATE_STATUS, &result);
    if (!result) {
        std::vector<GLchar> program_log(1024, 0);
        glGetProgramInfoLog(program, program_log.size(), nullptr, program_log.data());
        printf("[error]: %s\n", program_log.data());
        return {0};
    }
    return {
        .program         = program,
        .vertex_shader   = vertex_shader,
        .fragment_shader = fragment_shader,
    };
}

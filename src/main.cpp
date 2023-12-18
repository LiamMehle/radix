#define __cplusplus 202003L
#include <cstdio>
#include <cstdint>
#include <cassert>
#include <cstring>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <optional>
#include "gl.h"
#include "utils.hpp"
#include "raii.cpp"
constexpr size_t WIDTH  = 800;
constexpr size_t HEIGHT = 600;

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

Shader compile_shader(char const* const src, GLenum const type) {
    Shader shader(type);
    GLint success;
    std::vector<GLchar> program_log(1024, 0);
    GLint src_len = strlen(src);
    glShaderSource(shader, 1, &src, &src_len);
    glCompileShader(shader);
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, program_log.size(), nullptr, program_log.data());
        printf("[error]: %s\n", program_log.data());
        return 0;
    }
    return shader;
}

enum Direction {
    Left, Right
};

int main() {
    int status = 0;
    glfwSetErrorCallback(error_callback);
    GLFW glfw{};
    if (glfw != GLFW_TRUE) {
        puts("failed to initialize glfw");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    Window window = Window(WIDTH, HEIGHT, "Radix", nullptr, nullptr);
    if (!static_cast<GLFWwindow*>(window)) {
        puts("failed to create a window");
        return 2;
    }

    int frame_buffer_width, frame_buffer_height;
    glfwGetFramebufferSize(window, &frame_buffer_width, &frame_buffer_height);

    glfwMakeContextCurrent(window);

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
    VBO vbo{};
    VAO vao{};

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

    Program program{};
    Shader vertex_shader   = compile_shader(vertex_shader_src.c_str(),   GL_VERTEX_SHADER);
    Shader fragment_shader = compile_shader(fragment_shader_src.c_str(), GL_FRAGMENT_SHADER);
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
            return 6;
        }
    }

    GLint const x_offset_uniform = glGetUniformLocation(program, "x_offset");

    // enable vsync if present:
    glfwSwapInterval(1);
    Direction movement_direction = Left;
    float triangle_offset = 0;
    float constexpr triangle_max_offset = 0.7;
    float constexpr triangle_increment = 0.005;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        if (abs(triangle_offset) + triangle_increment > triangle_max_offset)
            movement_direction = movement_direction == Right ? Left : Right;

        triangle_offset += movement_direction == Right ? triangle_increment : -triangle_increment;

        glClearColor(.1f, .1f, .1f, 5.f);
        glClear(GL_COLOR_BUFFER_BIT);       

        glUseProgram(program);
        glBindVertexArray(vao);
        glUniform1f(x_offset_uniform, triangle_offset);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);

        glfwSwapBuffers(window);
    }

    return 0;
}
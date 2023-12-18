#include <cstdio>
#include <cstdint>
#include <cassert>
#include <cstring>
#include <vector>
#include "gl.h"
#include "raii.cpp"

constexpr size_t WIDTH  = 800;
constexpr size_t HEIGHT = 600;

void error_callback(int, const char* err_str) {
    printf("GLFW Error: %s\n", err_str);
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

    // glBindBuffer(GL_ARRAY_BUFFER, 0);                                             // unbinding the buffers (safety?)
    glBindVertexArray(0);                                                         //

    // shaders:
    GLchar const* const vertex_shader_src = "   \n\
#version 330                              \n\
layout (location=0) in vec3 pos;          \n\
void main() {                             \n\
    gl_Position = vec4(pos, 1.);          \n\
}                                         ";
    GLchar const* const fragment_shader_src = " \n\
#version 330                              \n\
out vec4 color;                           \n\
void main() {                             \n\
    color = vec4(4.f, 2.f, 8.f, 1.f);     \n\
}                                         ";

    Program program{};
    Shader vertex_shader   = compile_shader(vertex_shader_src,   GL_VERTEX_SHADER);
    Shader fragment_shader = compile_shader(fragment_shader_src, GL_FRAGMENT_SHADER);
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

    // enable vsync if present:
    glfwSwapInterval(1);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        glClearColor(.1f, .1f, .1f, 5.f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(program);
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);

        glfwSwapBuffers(window);
    }

    return 0;
}
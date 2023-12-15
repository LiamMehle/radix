#include <cstdio>
#include <cstdint>
#include "gl.h"
#include "raii.cpp"

constexpr size_t WIDTH  = 800;
constexpr size_t HEIGHT = 600;

void error_callback(int, const char* err_str) {
    printf("GLFW Error: %s\n", err_str);
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

    VBO vbo{};
    VAO vao{};

    glBindVertexArray(vao);                                                       // bind configuration object: remembers the global (buffer) state
    glBindBuffer(GL_ARRAY_BUFFER, vbo);                                           // bind the buffer to the slot for how it will be used
    glBufferData(GL_ARRAY_BUFFER, sizeof(verticies), verticies, GL_STATIC_DRAW);  // send data to the gpu (with usage hints)
    glVertexAttribPointer(0, 3, GL_FLOAT, false, 1, nullptr);                     // configure vbo metadata
    glEnableVertexAttribArray(0);                                                 // enable the config

    glBindBuffer(GL_ARRAY_BUFFER, 0);                                             // unbinding the buffers (safety?)
    glBindVertexArray(0);                                                         //

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        glClearColor(0.f, 0.f, 0.f, 0.f);
        glClear(GL_COLOR_BUFFER_BIT);

        glfwSwapBuffers(window);
    }

    return 0;
}
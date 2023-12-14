#include <cstdio>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cstdint>

constexpr size_t WIDTH  = 800;
constexpr size_t HEIGHT = 600;

int main() {
    int status = 0;
    if (glfwInit() != GLFW_TRUE) {
        puts("failed to initialize glfw");
        glfwTerminate();
        return 1;
    }
    glfwWindowHint(GLFW_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Radix", nullptr, nullptr);
    if (!window) {
        puts("failed to create a window");
        status = 2;
        goto window_fail;
    }

    int frame_buffer_width, frame_buffer_height;
    glfwGetFramebufferSize(window, &frame_buffer_width, &frame_buffer_height);

    glfwMakeContextCurrent(window);

    glewExperimental = GL_TRUE;

    if (glewInit() != GL_TRUE) {
        puts("failed to init glew");
        status = 3;
        goto window_fail;
    }

    glViewport(0, 0, frame_buffer_width, frame_buffer_height);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        glClearColor(0.f, 0.f, 0.f, 0.f);
        glClear(GL_COLOR_BUFFER_BIT);

        glfwSwapBuffers(window);
    }

window_fail:
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
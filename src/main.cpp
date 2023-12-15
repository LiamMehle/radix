#include <cstdio>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cstdint>

constexpr size_t WIDTH  = 800;
constexpr size_t HEIGHT = 600;

void error_callback(int, const char* err_str) {
    printf("GLFW Error: %s\n", err_str);
}

class VAO {
private:
    GLuint vao;
public:
    VAO() { glCreateVertexArrays(1, &this->vao); }
    VAO(GLuint const& other) { this->vao = other; }
    ~VAO() { glDeleteVertexArrays(1, &vao); }
    void operator=(VAO&) = delete;
    void operator=(VAO&& other) { this->vao = other; };

    operator GLuint() { return this->vao; }
};

class Window {
private:
    GLFWwindow* window;
public:
    Window(int width, int height, char const* const title, GLFWmonitor *monitor, GLFWwindow *share) {
        window = glfwCreateWindow(width, height, title, monitor, share); }
    Window(GLFWwindow *other) { this->window = other; }
    ~Window() { glfwDestroyWindow(this->window); }
    void operator=(VAO&) = delete;
    void operator=(VAO&& other) { this->window = other; };

    operator GLFWwindow*() { return this->window; }
};

class GLFW {
private:
    int status;
public:
    GLFW() { this->status = glfwInit(); }
    GLFW() { this->status = glfwInit(); }
    ~GLFW() { glfwTerminate(); }
    operator int() { return this->status; }
}

int main() {
    int status = 0;
    glfwSetErrorCallback(error_callback);
    if (glfwInit() != GLFW_TRUE) {
        puts("failed to initialize glfw");
        glfwTerminate();
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
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
        status = 3;
        goto window_fail;
    }

    glViewport(0, 0, frame_buffer_width, frame_buffer_height);

    VAO vao{};

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        glClearColor(0.f, 0.f, 0.f, 0.f);
        glClear(GL_COLOR_BUFFER_BIT);

        glfwSwapBuffers(window);
    }

    return 0;
}
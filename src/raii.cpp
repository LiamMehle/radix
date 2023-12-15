#include "gl.h"

class VAO {
private:
    GLuint vao;
public:
    VAO() { glGenVertexArrays(1, &this->vao); }
    VAO(GLuint const& other) { this->vao = other; }
    VAO(const VAO&) = default;
    ~VAO() { glDeleteVertexArrays(1, &vao); }
    void operator=(VAO&) = delete;
    void operator=(VAO&& other) { this->vao = other; };

    operator GLuint() { return this->vao; }
};
class VBO {
private:
    GLuint vbo;
public:
    VBO() { glCreateBuffers(1, &this->vbo); }
    VBO(GLuint const& other) { this->vbo = other; }
    VBO(const VBO&) = default;
    ~VBO() { glDeleteVertexArrays(1, &vbo); }
    void operator=(VBO&) = delete;
    void operator=(VBO&& other) { this->vbo = other; };

    operator GLuint() { return this->vbo; }
};

class Window {
private:
    GLFWwindow* window;
public:
    Window(int width, int height, char const* const title, GLFWmonitor *monitor, GLFWwindow *share) {
        window = glfwCreateWindow(width, height, title, monitor, share); }
    Window(GLFWwindow *other) { this->window = other; }
    Window(Window& other) = default;
    ~Window() { glfwDestroyWindow(this->window); }
    void operator=(Window&) = delete;
    void operator=(Window&& other) { this->window = other; };

    operator GLFWwindow*() const { return this->window; }
};

class Program {
private:
    GLuint program;
public:
    Program() {
        program = glCreateProgram(); }
    Program(Program& other) = default;
    ~Program() { glDeleteProgram(this->program); }
    void operator=(Program&) = delete;
    void operator=(Program&& other) { this->program = other; };

    operator GLuint() const { return this->program; }
};

class Shader {
private:
    GLuint program;
public:
    Shader(GLenum type) {
        program = glCreateShader(type); }
    Shader(Shader& other) = default;
    ~Shader() { glDeleteShader(this->program); }
    void operator=(Shader&) = delete;
    void operator=(Shader&& other) { this->program = other; };

    operator GLuint() const { return this->program; }
};

class GLFW {
private:
    int status;
public:
    GLFW() { this->status = glfwInit(); }
    ~GLFW() { glfwTerminate(); }
    operator int() { return this->status; }
};
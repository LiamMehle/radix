#pragma once
#include "gl.h"
#include "utils.hpp"
namespace raii {
    class VAO {
    private:
        GLuint vao;
    public:
        VAO() { glGenVertexArrays(1, &this->vao); debug_print("VAO() %d\n", this->vao); }
        VAO(const VAO&) = default;
        ~VAO() { glDeleteVertexArrays(1, &vao); debug_print("~VAO() %d\n", this->vao); }
        void operator=(VAO&) = delete;
        void operator=(VAO&& other) { this->vao = other; debug_print("VAO(VAO&&) %d\n", this->vao);};

        operator GLuint() const { return this->vao; }
    };
    class VBO {
    private:
        GLuint vbo;
    public:
        VBO() { glCreateBuffers(1, &this->vbo); debug_print("VBO() %d\n", this->vbo); }
        VBO(const VBO&) = default;
        ~VBO() { glDeleteVertexArrays(1, &vbo); debug_print("~VBO() %d\n", this->vbo); }
        void operator=(VBO&) = delete;
        void operator=(VBO&& other) { this->vbo = other; debug_print("VBO(VBO&&) %d\n", this->vbo); };

        operator GLuint() const { return this->vbo; }
    };

    class Window {
    private:
        GLFWwindow* window;
    public:
        Window(int width, int height, char const* const title, GLFWmonitor *monitor, GLFWwindow *share) {
            this->window = glfwCreateWindow(width, height, title, monitor, share);
        }
        // Window(Window other)  { this->window = other->window; }
        Window(Window&& other) { this->window = nullptr; swap(this->window, window); }
        ~Window() { glfwDestroyWindow(this->window); }

        operator GLFWwindow*() const { return this->window; }
    };

    class Program {
    private:
        GLint program;
    public:
        Program() { this->program = glCreateProgram(); }
        Program(Program&& other) { this->program = 0; swap(this->program, other.program); }
        ~Program() { glDeleteProgram(this->program); }

        operator GLint() const { return this->program; }
    };

    class Shader {
    private:
        GLint shader;
    public:
        Shader(GLenum type) { this->shader = glCreateShader(type); }
        Shader(Shader&& other) { this->shader = 0; swap(this->shader, other.shader); }
        ~Shader() { glDeleteShader(this->shader); }

        operator GLint() const { return this->shader; }
    };

    class GLFW {
    private:
        int status;
    public:
        GLFW() { this->status = glfwInit(); }
        ~GLFW() { glfwTerminate(); }
        operator int() { return this->status; }
    };
}
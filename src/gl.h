#pragma once
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/glx.h>
#include <X11/Xlib.h>
#ifndef NODEBUG
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
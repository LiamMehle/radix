#pragma once
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/glx.h>
#include <X11/Xlib.h>

static inline
GLenum print_gl_errors(char const* const where) {
    printf("%s:\n", where);
    GLenum error = GL_NO_ERROR;
    error = glGetError();
    if (error == GL_NO_ERROR)
        return GL_NO_ERROR;
    while (error != GL_NO_ERROR) {
        printf("\t%s\n", gluErrorString(error));
        error = glGetError();
    }
    return -1;
}
#pragma once
#include <GLFW/glfw3.h>
struct CursorPos {
    double x, y;
};
static inline
CursorPos get_cursor_pos(GLFWwindow* w) {
    CursorPos p;
    glfwGetCursorPos(w, &p.x, &p.y);
    return p;
}

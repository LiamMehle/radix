#pragma once
#include <GLFW/glfw3.h>

template<typename T>
static inline constexpr
auto max(T const a, T const b) -> T { return a > b ? a : b; }
template<typename T>
static inline constexpr
auto min(T const a, T const b) -> T { return a < b ? a : b; }
template<typename T>
static inline constexpr
auto abs(T const a) -> T { return a >= 0 ? a : -a; }

struct CursorPos { double x, y; };
static inline
CursorPos get_cursor_pos(GLFWwindow* w) {
    CursorPos p;
    int width, height;
    glfwGetCursorPos(w, &p.x, &p.y);
    glfwGetWindowSize(w, &width, &height);
    p.x /= width;   // normalization
    p.y /= height;
    p.x = max(p.x, 0.);
    p.y = max(p.y, 0.);
    p.x = min(p.x, 1.);
    p.y = min(p.y, 1.);
    return p;
}

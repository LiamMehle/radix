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

struct CursorPosition { float x, y; };
// gets the mouse location in device-normal space
static inline
CursorPosition get_cursor_pos(GLFWwindow* w) {
    int width, height;
    double x, y;
    glfwGetCursorPos(w, &x, &y);
    glfwGetWindowSize(w, &width, &height);
    x = (x / width ) * 2 - 1;  // conversion from screen-pixel space to device-normal space
    y = (y / height) * 2 - 1;
    x = max(x, -1.);
    y = max(y, -1.);
    x = min(x,  1.);
    y = min(y,  1.);
    return {
        .x=static_cast<float>(x),
        .y=static_cast<float>(y),
        };
}

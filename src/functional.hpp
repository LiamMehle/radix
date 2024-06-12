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
template<typename T>
static inline constexpr
void swap(T& a, T& b) {
    T temporary = b;
    b = a;
    a = temporary;
}
template<typename T>
static inline constexpr
auto clamp(T const& val, T const& min_value, T const& max_value) -> T {
    return max(min(val, max_value), min_value);
}

struct CursorPosition { float x, y; };
// gets the mouse location in device-normal space
static inline
CursorPosition get_cursor_pos(GLFWwindow* w) {
    int width, height;
    double x, y;
    glfwGetCursorPos(w, &x, &y);
    glfwGetWindowSize(w, &width, &height);
    x =   (x / width ) * 2 - 1;  // conversion from screen-pixel space to device-normal space
    y = -((y / height) * 2 - 1);
    return {
        .x=static_cast<float>(clamp(x, -1., 1.)),
        .y=static_cast<float>(clamp(y, -1., 1.)),
    };
}

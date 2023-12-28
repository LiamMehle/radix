#pragma once
#include "ros/ros.h"
#include "gl.h"
#include "raii.cpp"
#include <condition_variable>
#include <mutex>
#include <atomic>

void ros_event_loop(int argc, char** const argv, raii::Window const& window);

struct TriangleBuffer {
    raii::VBO vbo{};
    size_t triangle_count = 0;
    size_t capacity = 0;
    TriangleBuffer() = default;
    TriangleBuffer(TriangleBuffer const&) = default;
    TriangleBuffer& operator=(TriangleBuffer&) = default;
};

/*  semantics/conrtact:
- after state change: signal all waiters on condition variable update
- vbo points to either:
    - inactive buffer to be filled for next present
    - null, when inactive buffer is ready for present and is to be swapped out with active buffer
*/
struct ExposedRenderData {
    std::condition_variable update{};
    std::atomic<float*> mapped_buffer = nullptr;
    size_t triangle_count = 0;  // number of triangles written | input to render pipeline
    size_t capacity = 0;        // buffer's capacity           | output from render pipeline request
    size_t needed_capacity = 0; // needed capacity             | input into render pipeline
};

extern ExposedRenderData shared_render_data;
#pragma once
#include "ros/ros.h"
#include "gl.h"
#include "raii.cpp"
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <array>

void ros_event_loop(int argc, char** const argv);

struct TriangleBuffer {
    GLuint vbo            = 0;
    size_t triangle_count = 0;  // number of triangles stored
    size_t capacity       = 0;  // number of floats that can be stored
    float* mapping        = nullptr;
    float  zoom           = 0;
};

/*  semantics/conrtact:
- after state change: signal all waiters on condition variable update
- vbo points to either:
    - inactive buffer to be filled for next present
    - null, when inactive buffer is ready for present and is to be swapped out with active buffer
*/
struct ExposedRenderData {
    std::condition_variable update{};
    std::array<TriangleBuffer, 2> buffers;
    int active_buffer;                      // buffer currentry being drawn from
    std::atomic<bool> copy_in_progress;     // copy into inactive buffer in progress, also implies the ownership of the entire struct's resources
    size_t needed_capacity = 0;             // number of floats needed for full copy
};

extern ExposedRenderData shared_render_data;
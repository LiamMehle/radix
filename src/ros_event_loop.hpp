#pragma once
#include "ros/ros.h"
#include "gl.h"
#include "raii.cpp"
#include <condition_variable>
#include <mutex>
#include <atomic>

void ros_event_loop(int argc, char** const argv, raii::Window const& window);

struct GLBufferObject {
    GLuint vbo;
    size_t vertex_count;
};

struct ExposedRenderData {
    std::atomic<GLBufferObject*> inactive_buffer;
};

extern ExposedRenderData shared_render_data;
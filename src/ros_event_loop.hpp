#pragma once
#include "ros/ros.h"
#include "gl.h"
#include "raii.cpp"
#include <condition_variable>
#include <mutex>
#include <atomic>

void ros_event_loop(int argc, char** const argv, raii::Window const& window);

struct ExposedRenderData {
    std::condition_variable point_cloud_state_updated;
    std::mutex              point_cloud_mutex;
    std::vector<float>      point_cloud_triangles;   // vector of triangles (flattened into floats)
    std::atomic<bool>       point_cloud_is_fresh;
};

extern ExposedRenderData shared_render_data;
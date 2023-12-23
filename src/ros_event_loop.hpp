#pragma once
#include "ros/ros.h"
#include "gl.h"
#include "raii.cpp"
#include <condition_variable>
#include <mutex>
#include <atomic>

void ros_event_loop(int argc, char** const argv, raii::Window const& window);

extern std::condition_variable point_cloud_updated;
extern std::mutex              point_cloud_mutex;
extern std::vector<float>      point_cloud_triangles;
extern std::atomic<bool>       point_cloud_is_fresh;
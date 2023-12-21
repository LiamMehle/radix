#pragma once
#include "ros/ros.h"
#include "gl.h"
#include "raii.cpp"

void ros_event_loop(int argc, char** const argv, raii::Window const& window);
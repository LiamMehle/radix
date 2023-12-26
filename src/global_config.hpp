#pragma once
#include <cstdint>
/*
    Base rate at which everything refreshes
    This program is based on polling. This is the base rate, mainly setting the rate a which the display refreshes.
*/
int constexpr global_base_rate = 60;
constexpr std::size_t WIDTH  = 800;
constexpr std::size_t HEIGHT = 600;
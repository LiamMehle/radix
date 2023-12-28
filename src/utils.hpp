#pragma once
#ifdef DEBUG_OUTPUT
#define debug_print printf
#else
#define debug_print(...)
#endif
template<typename T>
static inline constexpr
void swap(T& a, T& b) {
    T temporary = b;
    b = a;
    a = temporary;
}
template<typename T>
static inline constexpr
auto max(T const a, T const b) -> T { return a > b ? a : b; }
template<typename T>
static inline constexpr
auto min(T const a, T const b) -> T { return a < b ? a : b; }
template<typename T>
static inline constexpr
auto abs(T const a) -> T { return a >= 0 ? a : -a; }
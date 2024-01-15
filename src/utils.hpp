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
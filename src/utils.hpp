#pragma once
template<typename T>
static inline constexpr
void swap(T& a, T& b) {
    T temporary = b;
    b = a;
    a = temporary;
}
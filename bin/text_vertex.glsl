#version 330
layout (location=0) in vec4 data;
out vec2 text_coords;

void main() {
    text_coords = data.zw;
    gl_Position = vec4(data.xy, 0.f, 1.f);
}
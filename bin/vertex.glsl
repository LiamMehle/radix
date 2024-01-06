#version 330
layout (location=0) in vec3 pos;
uniform float zoom;
void main() {
    vec2 pos = pos.xy * zoom;
    gl_Position = vec4(pos, 1.f, 1.);
}
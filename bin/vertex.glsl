#version 330
layout (location=0) in vec3 pos;
uniform float z;
uniform float zoom;
void main() {
    vec2 pos = pos.xy * zoom;
    gl_Position = vec4(pos, z, 1.);
}
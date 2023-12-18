#version 330
uniform float x_offset;
layout (location=0) in vec3 pos;
void main() {
    vec3 new_pos = pos;
    new_pos.x += x_offset;
    gl_Position = vec4(new_pos, 1.);
}
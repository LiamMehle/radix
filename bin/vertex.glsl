#version 330
uniform float x_max;
uniform float y_max;
layout (location=0) in vec3 pos;
void main() {
    vec3 new_pos = pos;
    float z = 0.;
    vec2 pos = pos.xy / x_max;
    gl_Position = vec4(pos, z, 1.);
}
#version 330
uniform float x_max;
uniform float y_max;
layout (location=0) in vec3 pos;
void main() {
    vec3 new_pos = pos;
    new_pos.z = 0.;
    gl_Position = vec4(new_pos, 1.);
}
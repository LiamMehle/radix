#version 330
layout (location=0) in vec2 pos;
void main() {
    float z = 0.;
    gl_Position = vec4(pos, 0., 1.);
}
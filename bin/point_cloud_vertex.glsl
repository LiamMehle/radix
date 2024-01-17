#version 330
layout (location=0) in vec3 pos;
void main() {
    float z = 0.;
    vec2 pos = pos.xy;
    gl_Position = vec4(pos, 0., 1.);
}
#version 330
layout (location=0) in vec3 pos;
out float heights;
void main() {
    heights = pos.z;
    gl_Position = vec4(pos.xy * 4, pos.z, 1.);
}

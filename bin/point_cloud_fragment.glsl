#version 330
out vec4 color;
in float height;
void main() {
    color = vec4(1.f-height, abs(.5f-height), height, 1.f);
}
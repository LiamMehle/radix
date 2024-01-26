#version 330
out vec4 out_color;
uniform sampler2D text_bitmap;
uniform vec3 color;
uniform float top;
uniform float left;
uniform float bottom;
uniform float right;
void main() {
    vec2 min_coords = vec2(right, top);
    vec2 max_coords = vec2(left, bottom);
    vec2 text_relative_position = (gl_FragCoord.xy - min_coords) / (max_coords-min_coords);
    float alpha = texture(text_bitmap, text_relative_position).x;
    out_color = vec4(color, alpha);
}
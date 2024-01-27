#version 330

out vec4 out_color;
in vec2 text_coords;

uniform sampler2D text_bitmap;

bool isnan( float val )
{
  return ( val < 0.0 || 0.0 < val || val == 0.0 ) ? false : true;
}

void main() {
    vec4 alpha = texture(text_bitmap, text_coords);
    out_color = vec4(text_coords, 1.f, 1.f) * alpha;
}
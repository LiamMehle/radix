#version 330

out vec4 out_color;
in vec2 text_coords;

uniform sampler2D text_bitmap;

bool isnan( float val )
{
  return ( val < 0.0 || 0.0 < val || val == 0.0 ) ? false : true;
}

void main() {
    float alpha = texture(text_bitmap, text_coords).x;
    // if (alpha == 0.f)
    //   discard;
    out_color = vec4(text_coords, 1.f, alpha);
}
#version 330

out vec4 out_color;
in vec2 text_coords;

uniform sampler2D text_bitmap;
uniform int pitch;

bool isnan( float val ) {
  return ( val < 0.0 || 0.0 < val || val == 0.0 ) ? false : true;
}

void main() {
  float THRESHOLD = 0.f;
  float alpha = texture(text_bitmap, text_coords).x;
//  if (alpha <= THRESHOLD) discard;

  vec4 color = vec4(text_coords, 0.f, alpha);
//  color = color.xzxz
//            + color.zyyz
//            + color.zzzw;
  out_color = color; //vec4(text_coords, alpha, 1.f);
}
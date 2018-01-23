#version 400

in vec4 color_vert;
out vec4 frag_color;

void main () {
  frag_color = color_vert;
} 
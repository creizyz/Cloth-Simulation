#version 400

in vec3 position;
in vec4 color;

uniform mat4 VPMatrix;

out vec4 color_vert;

void main () {
  gl_Position = vec4(position, 1.0) * VPMatrix;
  color_vert = color;
} 
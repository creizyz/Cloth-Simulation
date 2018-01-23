#version 400

in vec3 position;
in float strain;

uniform mat4 VPMatrix;

out vec4 color_vert;

void main () {
  gl_Position = vec4(position, 1.0) * VPMatrix;
  color_vert = vec4(strain, 0.0, 0.0, 1.0);
  //color_vert = vec4(strain, 1.0 - strain, 0.0, 1.0);
  //color_vert = vec4(1.0, 1.0, 1.0, 1.0);
}
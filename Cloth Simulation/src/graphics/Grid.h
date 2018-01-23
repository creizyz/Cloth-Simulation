#ifndef __H__GRID__
#define __H__GRID__

#include "../maths/math.h"
#include "../graphics/openGL.h"

struct Grid
{
  gl::GLuint program;
  gl::GLint  colorUniform;
  float color[4];

  gl::GLuint VAO;
  gl::GLuint VBO;
  gl::GLuint IBO;
  size_t drawCount;

  bool initialised;

  Grid();
  ~Grid();

  void init(gl::GLuint shaderProgram, const math::vec3 & gridPos, const math::vec3 & axisA, const math::vec3 & axisB, float sizeSquare, size_t nbrOfSquare, float r, float g, float b, float a, bool bothDirection = true);
  void render(const math::mat4 & projMatrix);
};

#endif //__H__GRID__
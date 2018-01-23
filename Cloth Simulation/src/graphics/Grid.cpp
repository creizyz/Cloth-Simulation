#include "Grid.h" 

#include "../common/macro.h"

Grid::Grid() : VAO(0), VBO(0), IBO(0), drawCount(0), colorUniform(0), initialised(false)
{
  color[0] = 0.f;
  color[1] = 0.f;
  color[2] = 0.f;
  color[3] = 0.f;
}
Grid::~Grid()
{
  if (initialised)
  {
    gl::glDeleteBuffers(1, &IBO);
    gl::glDeleteBuffers(1, &VBO);
    gl::glDeleteVertexArrays(1, &VAO);
  }
}

void Grid::init(gl::GLuint shaderProgram, const math::vec3 & gridPos, const math::vec3 & axisA, const math::vec3 & axisB, float sizeSquare, size_t nbrOfSquare, float r, float g, float b, float a, bool bothDirection)
{
  if (!initialised)
  {
    initialised = true;
    program = shaderProgram;
    colorUniform = gl::glGetUniformLocation(program, "color");

    color[0] = r;
    color[1] = g;
    color[2] = b;
    color[3] = a;

    math::vec3 offsetA = axisA * sizeSquare;
    math::vec3 offsetB = axisB * sizeSquare;

    math::vec3 maxOffsetA = axisA * (sizeSquare * nbrOfSquare);
    math::vec3 maxOffsetB = axisB * (sizeSquare * nbrOfSquare);

    //bothDirection = false;
    drawCount = 4 * nbrOfSquare;
    size_t nbrOfPointPerPass = 4;
    if (bothDirection)
    {
      drawCount *= 2;
      nbrOfPointPerPass *= 2;
    }
    gl::GLfloat * points = new gl::GLfloat[3 * drawCount]; // 2 vertex * nbrLine = 2 * 3 * nbrLine = 2 * 3 * 2 * nbOfSteps
    for (size_t n = 0; n < nbrOfSquare; n++)
    {
      size_t offset_n = nbrOfPointPerPass * 3 * n;
      math::vec3 A1;
      math::vec3 B1;
      if (bothDirection)
      {
        A1 = gridPos + (offsetA * n) - maxOffsetB;
        B1 = gridPos + (offsetB * n) - maxOffsetA;
      }
      else
      {
        A1 = gridPos + (offsetA * n);
        B1 = gridPos + (offsetB * n);
      }
      math::vec3 A2 = gridPos + (offsetA * n) + maxOffsetB;
      math::vec3 B2 = gridPos + (offsetB * n) + maxOffsetA;

      points[offset_n] = A1.x;
      points[offset_n + 1] = A1.y;
      points[offset_n + 2] = A1.z;

      points[offset_n + 3] = A2.x;
      points[offset_n + 4] = A2.y;
      points[offset_n + 5] = A2.z;

      points[offset_n + 6] = B1.x;
      points[offset_n + 7] = B1.y;
      points[offset_n + 8] = B1.z;

      points[offset_n + 9] = B2.x;
      points[offset_n + 10] = B2.y;
      points[offset_n + 11] = B2.z;

      if (bothDirection)
      {
        A1 = gridPos - (offsetA * n) - maxOffsetB;
        B1 = gridPos - (offsetB * n) - maxOffsetA;
        A2 = gridPos - (offsetA * n) + maxOffsetB;
        B2 = gridPos - (offsetB * n) + maxOffsetA;

        points[offset_n + 12] = A1.x;
        points[offset_n + 13] = A1.y;
        points[offset_n + 14] = A1.z;

        points[offset_n + 15] = A2.x;
        points[offset_n + 16] = A2.y;
        points[offset_n + 17] = A2.z;

        points[offset_n + 18] = B1.x;
        points[offset_n + 19] = B1.y;
        points[offset_n + 20] = B1.z;

        points[offset_n + 21] = B2.x;
        points[offset_n + 22] = B2.y;
        points[offset_n + 23] = B2.z;
      }
    }
    gl::glGenVertexArrays(1, &VAO);
    gl::glGenBuffers(1, &VBO);

    gl::glBindVertexArray(VAO);
    gl::glBindBuffer(gl::GL_ARRAY_BUFFER, VBO);
    gl::glBufferData(gl::GL_ARRAY_BUFFER, 3 * drawCount * sizeof(gl::GLfloat), points, gl::GL_STATIC_DRAW);

    gl::GLint position_attribute = gl::glGetAttribLocation(program, "position");
    gl::glVertexAttribPointer(position_attribute, 3, gl::GL_FLOAT, gl::GL_FALSE, 0, 0);
    gl::glEnableVertexAttribArray(position_attribute);
    gl::glBindVertexArray(0);

    delete[](points);
  }
}
void Grid::render(const math::mat4 & projMatrix)
{
  DBG_ASSERT(initialised);
  if (initialised)
  {
    gl::glUseProgram(program);

    gl::GLint matrixUniform = gl::glGetUniformLocation(program, "VPMatrix");
    if (matrixUniform != -1) gl::glUniformMatrix4fv(matrixUniform, 1, gl::GL_FALSE, projMatrix.data);
    if (colorUniform != -1) gl::glUniform4f(colorUniform, color[0], color[1], color[2], color[3]);

    gl::glBindVertexArray(VAO);
    gl::glDrawArrays(gl::GL_LINES, 0, drawCount);
  }
}
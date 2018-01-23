#ifndef __H__CLOTH__
#define __H__CLOTH__

#include "../graphics/openGL.h"
#include "motion_system.h"

#define USE_IMPULSE
#define USE_IMPULSE_TO_FIX_POINTS
//#define UPDATE_ALL_AT_ONCE

extern gl::GLuint shaderProgram;

struct Cloth
{
  LinearMotionSystem & _lms;
  bool initialised;

  // STATIC VARIABLES
  float color[4];
  float density;
  float triangleStiffness;
  float edgeStiffness;
  float thickness;
  size_t nbrOfPoints;
  size_t nbrOfTriangles;
  size_t nbrOfEdges;
  size_t width;

  size_t * triangles;
  size_t * edges;
  float * invNbrAdjTriangles;
  float * invNbrAdjEdges;
  float * mass;

  // DYNAMIC VARIABLES
  math::vec3 * posInit;
  size_t *     posCur;
  gl::GLfloat * vertex_t; // only positions, use IBO
  gl::GLfloat * vertex_e; // only positions, no IBO because 1 color per edge

#ifdef UPDATE_ALL_AT_ONCE
  math::vec3 * correction;
#endif

  // OPENGL VARIABLES
  gl::GLuint program[2];
  gl::GLuint VAO[2];
  gl::GLuint VBO[2];
  gl::GLuint IBO; // <- used by triangles only

  void allocateSpace(size_t size_h, size_t size_w);
  void setColor(float r, float g, float b, float a);
  void updateMass();
  void setDensity(float _density);
  void setStiffness(float triangle, float edge);
  void setThickness(float _thickness);
  void updateNbrTriangleAndEdgePerPoint();
  Cloth(LinearMotionSystem & lms);
  Cloth(LinearMotionSystem & lms, const math::vec3 & pos, const math::vec3 & axis_h, const math::vec3 & axis_w, size_t size_h, size_t size_w, float step_h, float step_w);
  ~Cloth();

  void initGL(gl::GLuint uniformColorProgram, gl::GLuint strainColorProgram);
  void render(const math::mat & projMatrix, bool renderTriangles = true, bool renderEdges = true);

  void applyTriangleShapeMatching(size_t iTriangle);
  void triangle2DCorrection(size_t iTriangle);
  void edgeCorrection(size_t iEdge);
  void update();
};

#endif //__H__CLOTH__
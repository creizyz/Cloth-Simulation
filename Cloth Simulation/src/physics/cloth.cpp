#include "cloth.h"

#include <math.h>
#include <vector>

#include "../maths/math.h"
#include "../common/macro.h"

void Cloth::allocateSpace(size_t size_h, size_t size_w)
{
  nbrOfPoints = size_h * size_w;
  nbrOfTriangles = (2 * (size_w - 1) * (size_h - 1));
  nbrOfEdges = ((size_h - 1) * size_w + (size_w - 1) * size_h) + ((size_h - 1) * (size_w - 1));

  triangles = new size_t[nbrOfTriangles * 3]; // 3 indices per triangle
  edges = new size_t[nbrOfEdges * 2]; // 2 indices per triangle

  invNbrAdjTriangles = new float[nbrOfPoints];
  invNbrAdjEdges = new float[nbrOfPoints];
  mass = new float[nbrOfPoints];
  posInit = new math::vec3[nbrOfPoints];
  posCur = new size_t[nbrOfPoints];

  vertex_t = new gl::GLfloat[nbrOfPoints * 3]; // only positions, use static IBO
  vertex_e = new gl::GLfloat[nbrOfEdges * 2 * 4]; // only positions, no IBO because 1 color per edge (4) because x, y, z, strain
#ifdef UPDATE_ALL_AT_ONCE
  correction = new math::vec3[nbrOfPoints];
#endif
}
void Cloth::setColor(float r, float g, float b, float a)
{
  color[0] = r;
  color[1] = g;
  color[2] = b;
  color[3] = a;
}
void Cloth::updateMass()
{
  memset(mass, 0, nbrOfPoints * sizeof(float));
  for (size_t n = 0; n < nbrOfTriangles; n++)
  {
    size_t indexPoint[] = {
      triangles[3 * n],
      triangles[3 * n + 1],
      triangles[3 * n + 2]
    };
    math::vec3 pos[] = {
      posInit[indexPoint[0]],
      posInit[indexPoint[1]],
      posInit[indexPoint[2]]
    };
    math::vec3 side[] = {
      pos[1] - pos[0],
      pos[2] - pos[0]
    };
    math::vec3 crossprod = math::vec3::cross(side[0], side[1]);
    float triangleArea = sqrt(crossprod.length()) / 2.f;
    DBG_VALID_FLOAT(triangleArea);
    float triangleMass = density * triangleArea;
    DBG_VALID_FLOAT(triangleMass);
    float massPerPoint = triangleMass / 3.f;
    if (massPerPoint == 0.f) DBG_HALT;
    DBG_VALID_FLOAT(massPerPoint);
    mass[indexPoint[0]] += massPerPoint;
    mass[indexPoint[1]] += massPerPoint;
    mass[indexPoint[2]] += massPerPoint;
  }
  for (size_t n = 0; n < nbrOfPoints; n++)
  {
    _lms.set_mass(posCur[n], mass[n]);
  }
}
void Cloth::setDensity(float _density)
{
  density = _density;
  updateMass();
}
void Cloth::setStiffness(float triangle, float edge)
{
  triangleStiffness = triangle;
  edgeStiffness = edge;
}
void Cloth::setThickness(float _thickness)
{
  thickness = _thickness;
}
void Cloth::updateNbrTriangleAndEdgePerPoint()
{
  int * countTriangle = new int[nbrOfPoints];
  int * countEdge = new int[nbrOfPoints];
  memset(countTriangle, 0, nbrOfPoints * sizeof(int));
  memset(countEdge, 0, nbrOfPoints * sizeof(int));
  for (size_t n = 0; n < nbrOfTriangles * 3; n += 3)
  {
    countTriangle[triangles[n]]++;
    countTriangle[triangles[n + 1]]++;
    countTriangle[triangles[n + 2]]++;
  }
  for (size_t n = 0; n < nbrOfEdges * 2; n += 2)
  {
    countEdge[edges[n]]++;
    countEdge[edges[n + 1]]++;
  }
  for (size_t n = 0; n < nbrOfPoints; n++)
  {
    if (countTriangle[n] != 0) invNbrAdjTriangles[n] = 1.f / countTriangle[n];
    else invNbrAdjTriangles[n] = 1.f;

    if (countEdge[n] != 0) invNbrAdjEdges[n] = 1.f / countEdge[n];
    else invNbrAdjEdges[n] = 1.f;
  }
}
Cloth::Cloth(LinearMotionSystem & lms) :
  _lms(lms),
  initialised(false),
  triangles(nullptr),
  edges(nullptr),
  invNbrAdjTriangles(nullptr),
  invNbrAdjEdges(nullptr),
  mass(nullptr),
  posInit(nullptr),
  posCur(nullptr),
  vertex_t(nullptr),
  vertex_e(nullptr),
#ifdef UPDATE_ALL_AT_ONCE
  correction(nullptr),
#endif
  density(0),
  triangleStiffness(0),
  edgeStiffness(0),
  nbrOfPoints(0),
  nbrOfTriangles(0),
  nbrOfEdges(0)
{

}
Cloth::Cloth(LinearMotionSystem & lms, const math::vec3 & pos, const math::vec3 & axis_h, const math::vec3 & axis_w, size_t size_h, size_t size_w, float step_h, float step_w) : Cloth(lms)
{
  width = size_w;
  allocateSpace(size_h, size_w);
  // CREATE POINTS
  math::vec3 stepRow = axis_h * step_h;
  math::vec3 stepCol = axis_w * step_w;
  for (size_t row = 0; row < size_h; row++)
  {
    for (size_t col = 0; col < size_h; col++)
    {
      math::vec3 pointPos = pos + (row * stepRow) + (col * stepCol);
      size_t n = (row * size_w) + col;
      posInit[n] = pointPos;
      posCur[n] = _lms.new_linear_data(pointPos, math::vec3(0.f, 0.f, 0.f), 1.f); // <- we will update the mass later
    }
  }
  // CREATE EDGES AND TRIANGLES LISTS
  size_t i = 0;
  for (size_t row = 0; row < size_h - 1; row++)
    for (size_t col = 0; col < size_w - 1; col++)
    {
      size_t a = (row * size_w) + col; // <- current index in points
      size_t b = a + 1;
      size_t c = a + size_w + 1;
      size_t d = a + size_w;
      edges[i] = a; // <- add  edge a - b
      edges[i + 1] = b;
      edges[i + 2] = a; // <- add edge a - c
      edges[i + 3] = c;
      edges[i + 4] = a; // <- add edge a - d
      edges[i + 5] = d;
      triangles[i] = a;
      triangles[i + 1] = b;
      triangles[i + 2] = c;
      triangles[i + 3] = a;
      triangles[i + 4] = c;
      triangles[i + 5] = d;
      i += 6;
    }
  // COMPLETE EDGE LIST
  size_t lastRow = size_h - 1;
  size_t lastCol = size_w - 1;
  for (size_t row = 0; row < size_h - 1; row++)
  {
    size_t a = (row * size_w) + lastCol;
    size_t b = a + size_w;
    edges[i] = a;
    edges[i + 1] = b;
    i += 2;
  }
  for (size_t col = 0; col < size_w - 1; col++)
  {
    size_t a = (lastRow * size_w) + col;
    size_t b = a + 1;
    edges[i] = a;
    edges[i + 1] = b;
    i += 2;
  }
  updateNbrTriangleAndEdgePerPoint();
}
Cloth::~Cloth()
{
  if (initialised)
  {
    gl::glDeleteVertexArrays(2, VAO);
    gl::glDeleteBuffers(2, VBO);
    gl::glDeleteBuffers(1, &IBO);
  }
  for (size_t i = 0; i < nbrOfPoints; i++) _lms.free_linear_data(posCur[i]);
  SAFE_DELETE_TAB(triangles);
  SAFE_DELETE_TAB(edges);
  SAFE_DELETE_TAB(invNbrAdjTriangles);
  SAFE_DELETE_TAB(invNbrAdjEdges);
  SAFE_DELETE_TAB(mass);
  SAFE_DELETE_TAB(posInit);
  SAFE_DELETE_TAB(posCur);
  SAFE_DELETE_TAB(vertex_t);
  SAFE_DELETE_TAB(vertex_e);
#ifdef UPDATE_ALL_AT_ONCE
  SAFE_DELETE_TAB(correction);
#endif
}

void Cloth::initGL(gl::GLuint uniformColorProgram, gl::GLuint strainColorProgram)
{
  if (!initialised)
  {
    initialised = true;
    program[0] = uniformColorProgram;
    program[1] = strainColorProgram;

    gl::glGenVertexArrays(2, VAO);
    gl::glGenBuffers(2, VBO);
    gl::glGenBuffers(1, &IBO);

    // TRIANGLES
    gl::glBindVertexArray(VAO[0]);
    gl::glBindBuffer(gl::GL_ARRAY_BUFFER, VBO[0]);
    gl::glBufferData(gl::GL_ARRAY_BUFFER, 3 * nbrOfPoints * sizeof(gl::GLfloat), posInit, gl::GL_DYNAMIC_DRAW);

    gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, IBO);
    gl::glBufferData(gl::GL_ELEMENT_ARRAY_BUFFER, 3 * nbrOfTriangles * sizeof(gl::GLuint), triangles, gl::GL_STATIC_DRAW);

    gl::GLint position_attribute = gl::glGetAttribLocation(program[0], "position");
    gl::glVertexAttribPointer(position_attribute, 3, gl::GL_FLOAT, gl::GL_FALSE, 0, 0);
    gl::glEnableVertexAttribArray(position_attribute);
    // EDGES
    gl::glBindVertexArray(VAO[1]);
    gl::glBindBuffer(gl::GL_ARRAY_BUFFER, VBO[1]);
    gl::glBufferData(gl::GL_ARRAY_BUFFER, 8 * nbrOfEdges * sizeof(gl::GLfloat), vertex_e, gl::GL_DYNAMIC_DRAW);

    gl::GLint position_attribute2 = gl::glGetAttribLocation(program[1], "position");
    //gl::glVertexAttribPointer(position_attribute2, 3, gl::GL_FLOAT, gl::GL_FALSE, 0, 0);
    gl::glVertexAttribPointer(position_attribute2, 3, gl::GL_FLOAT, gl::GL_FALSE, 4 * sizeof(gl::GLfloat), 0);
    gl::glEnableVertexAttribArray(position_attribute2);
    gl::GLint strain_attribute = gl::glGetAttribLocation(program[1], "strain");
    if (strain_attribute != -1) gl::glVertexAttribPointer(strain_attribute, 1, gl::GL_FLOAT, gl::GL_FALSE, 4 * sizeof(gl::GLfloat), (void*)(3 * sizeof(gl::GLfloat)));
    if (strain_attribute != -1) gl::glEnableVertexAttribArray(strain_attribute);
    gl::glBindVertexArray(0);
  }
}
void Cloth::render(const math::mat & projMatrix, bool renderTriangles, bool renderEdges)
{
  if (renderTriangles)
  {
    gl::glUseProgram(program[0]);

    gl::GLint matrixUniform = gl::glGetUniformLocation(program[0], "VPMatrix");
    if (matrixUniform != -1) gl::glUniformMatrix4fv(matrixUniform, 1, gl::GL_FALSE, projMatrix.data);
    gl::GLint colorUniform = gl::glGetUniformLocation(program[0], "color");
    if (colorUniform != -1) gl::glUniform4f(colorUniform, color[0], color[1], color[2], color[3]);

    for (size_t n = 0; n < nbrOfPoints; n++)
    {
      size_t i = 3 * n;
      math::vec3 vec = _lms.get_linear_position(posCur[n]);
      vertex_t[i] = vec.x;
      vertex_t[i + 1] = vec.y;
      vertex_t[i + 2] = vec.z;
      //if (n == 0) DBG_HALT;
    }
    gl::glBindVertexArray(VAO[0]);
    gl::glBindBuffer(gl::GL_ARRAY_BUFFER, VBO[0]);
    gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, IBO);
    gl::glBufferSubData(gl::GL_ARRAY_BUFFER, NULL, 3 * nbrOfPoints * sizeof(gl::GLfloat), vertex_t);
    gl::glDrawElements(gl::GL_TRIANGLES, 3 * nbrOfTriangles, gl::GL_UNSIGNED_INT, 0);
    gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, 0);
    gl::glBindBuffer(gl::GL_ARRAY_BUFFER, 0);
    gl::glBindVertexArray(0);
  }
  DBG_ASSERT(initialised);
  if (initialised)
  {
    if (renderEdges)
    {
      gl::glUseProgram(program[1]);

      gl::GLint matrixUniform = gl::glGetUniformLocation(program[0], "VPMatrix");
      if (matrixUniform != -1) gl::glUniformMatrix4fv(matrixUniform, 1, gl::GL_FALSE, projMatrix.data);

      for (size_t n = 0; n < nbrOfEdges; n++)
      {
        size_t indexPoint[] = {
          edges[2 * n],
          edges[2 * n + 1]
        };
        math::vec3 pos[] = {
          _lms.get_linear_position(indexPoint[0]),
          _lms.get_linear_position(indexPoint[1])
        };
        math::vec3 ini[] = {
          posInit[indexPoint[0]],
          posInit[indexPoint[1]],
        };
        float strain = abs(((pos[1] - pos[0]).length() / (ini[1] - ini[0]).length()) - 1.f);
        size_t i = 8 * n;
        //size_t i = 6 * n;

        vertex_e[i] = pos[0].x;
        vertex_e[i + 1] = pos[0].y;
        vertex_e[i + 2] = pos[0].z;
        vertex_e[i + 3] = strain;

        vertex_e[i + 4] = pos[1].x;
        vertex_e[i + 5] = pos[1].y;
        vertex_e[i + 6] = pos[1].z;
        vertex_e[i + 7] = strain;
      }
      gl::glBindVertexArray(VAO[1]);
      gl::glBindBuffer(gl::GL_ARRAY_BUFFER, VBO[1]);
      gl::glBufferSubData(gl::GL_ARRAY_BUFFER, NULL, 8 * nbrOfEdges * sizeof(gl::GLfloat), vertex_e);
      gl::glDrawArrays(gl::GL_LINES, 0, 2 * nbrOfEdges);
      gl::glBindBuffer(gl::GL_ARRAY_BUFFER, 0);
      gl::glBindVertexArray(0);
    }
  }
}

void Cloth::applyTriangleShapeMatching(size_t iTriangle)
{
  size_t indexPoint[3] = {
    triangles[3 * iTriangle],
    triangles[3 * iTriangle + 1],
    triangles[3 * iTriangle + 2]
  };
  // 0 : current, 1 : initial
  math::vec3 vec[2][3] = {
    {
      _lms.get_linear_position(indexPoint[0]),
      _lms.get_linear_position(indexPoint[1]),
      _lms.get_linear_position(indexPoint[2])
    },
    {
      posInit[indexPoint[0]],
      posInit[indexPoint[1]],
      posInit[indexPoint[2]]
    }
  };
  float invTotalMass = 1.f / (mass[indexPoint[0]] + mass[indexPoint[1]] + mass[indexPoint[2]]);
  math::vec3 cm[2];
  math::mat projMat[2];
  math::mat state[2];
  for (size_t n = 0; n < 2; n++)
  {
    math::vec3 side[] = { vec[n][1] - vec[n][0], vec[n][2] - vec[n][0] };
    math::vec3 normal = math::vec3::cross(side[0], side[1]).normalized();
    math::vec3 axis[] = { side[0].normalized(), math::vec3::cross(side[0], normal).normalized() };
    cm[n] = ((vec[n][0] * mass[indexPoint[0]]) + (vec[n][1] * mass[indexPoint[1]]) + (vec[n][2] * mass[indexPoint[2]])) * invTotalMass;

    float data_ProjMat[] = {
      axis[0].x, axis[0].y, axis[0].z,
      axis[1].x, axis[1].y, axis[1].z
    };
    projMat[n] = math::mat(2, 3, data_ProjMat);
  }

  math::mat def(2, 2);
  def *= 0.f;
  for (size_t n = 0; n < 3; n++)
  {
    math::mat x_i = projMat[0] * math::mat(vec[0][n] - cm[0]);
    math::mat x_i_0 = projMat[1] * math::mat(vec[1][n] - cm[1]);
    def += (x_i * math::mat::transpose(x_i_0)) * mass[indexPoint[n]];
  }
  float rot = 0.f;
  if (def[0][0] != 0.f) rot = std::atan2(def[0][1], def[0][0]);
  float cosR = cos(rot), sinR = sin(rot);
  float data_nDef[] = {
    cosR, sinR, def[0][2],
    -sinR, cosR, def[1][2]
  };
  math::mat nDef(2, 3, data_nDef);

  math::mat stateInit(3, 3);
  for (size_t n = 0; n < 9; n++)
    if (n < 6) stateInit.data[n] = state[1].data[n];
    else stateInit.data[n] = 1.f;

    math::mat goal = nDef * stateInit;
    math::mat ofst = (goal - state[0]) * triangleStiffness;

    for (size_t n = 0; n < 3; n++) // <- project if back into 3D space
    {
      float u = ofst[0][n] * invNbrAdjTriangles[indexPoint[n]];
      float v = ofst[1][n] * invNbrAdjTriangles[indexPoint[n]];
      math::vec3 correct((projMat[0][0][0] * u) + (projMat[0][1][0] * v), (projMat[0][0][1] * u) + (projMat[0][1][1] * v), (projMat[0][0][2] * u) + (projMat[0][1][2] * v));
#ifdef UPDATE_ALL_AT_ONCE
      correction[indexPoint[n]] += correct;
#else
      _lms.move_linear_position(posCur[indexPoint[n]], correct);
#endif
    }
}
void Cloth::triangle2DCorrection(size_t iTriangle)
{
  size_t indexPoint[3] = {
    triangles[3 * iTriangle],
    triangles[3 * iTriangle + 1],
    triangles[3 * iTriangle + 2]
  };
  // 0 : current, 1 : initial
  math::vec3 vec[2][3] = {
    {
      _lms.get_linear_position(indexPoint[0]),
      _lms.get_linear_position(indexPoint[1]),
      _lms.get_linear_position(indexPoint[2])
    },
    {
      posInit[indexPoint[0]],
      posInit[indexPoint[1]],
      posInit[indexPoint[2]]
    }
  };
  float invTotalMass = 1.f / (mass[indexPoint[0]] + mass[indexPoint[1]] + mass[indexPoint[2]]);
  math::vec3 cm[2];
  math::vec3 axis[2][2];
  float coord2D[2][3][2]; // initial/current | point | u/v
  for (size_t n = 0; n < 2; n++)
  {
    math::vec3 side[] = { vec[n][1] - vec[n][0], vec[n][2] - vec[n][0] };
    math::vec3 normal = math::vec3::cross(side[0], side[1]).normalized();
    axis[n][0] = side[0].normalized();
    axis[n][1] = math::vec3::cross(side[0], normal).normalized();
    cm[n] = ((vec[n][0] * mass[indexPoint[0]]) + (vec[n][1] * mass[indexPoint[1]]) + (vec[n][2] * mass[indexPoint[2]])) * invTotalMass;
    for (size_t i = 0; i < 3; i++)
    {
      coord2D[n][i][0] = math::vec3::dot(vec[n][i] - cm[n], axis[n][0]);
      coord2D[n][i][1] = math::vec3::dot(vec[n][i] - cm[n], axis[n][1]);
    }
  }

  for (size_t i = 0; i < 3; i++)
  {
    float length[2] = {
      sqrt(coord2D[0][i][0] * coord2D[0][i][0] + coord2D[0][i][1] * coord2D[0][i][1]),
      sqrt(coord2D[1][i][0] * coord2D[1][i][0] + coord2D[1][i][1] * coord2D[1][i][1])
    };

    float correctionFactor = ((length[1] - length[0]) / length[0]);
    float correct2D[] = {
      coord2D[0][i][0] * correctionFactor,
      coord2D[0][i][1] * correctionFactor
    };
    math::vec3 correct3D = ((axis[0][0] * correct2D[0]) + (axis[0][1] * correct2D[1])) * triangleStiffness * invNbrAdjTriangles[indexPoint[i]];
#ifdef UPDATE_ALL_AT_ONCE
#ifdef USE_IMPULSE
    correction[indexPoint[i]] += correct3D * INV_PHYSICS_TIME_STEP * mass[indexPoint[i]];
#else
    correction[indexPoint[i]] += correct3D;
#endif
#else
#ifdef USE_IMPULSE
    math::vec3 I = correct3D * INV_PHYSICS_TIME_STEP * mass[indexPoint[i]];
    _lms.apply_impulse(posCur[indexPoint[i]], I);
#else
    _lms.move_linear_position(posCur[indexPoint[i]], correct3D);
#endif
#endif
  }
}
void Cloth::edgeCorrection(size_t iEdge)
{
  size_t indexPoint[] = {
    edges[2 * iEdge],
    edges[2 * iEdge + 1]
  };
  float _mass[] = {
    mass[indexPoint[0]],
    mass[indexPoint[1]]
  };
  math::vec3 cur_pos[] = {
    _lms.get_linear_position(posCur[indexPoint[0]]),
    _lms.get_linear_position(posCur[indexPoint[1]])
  };
  math::vec3 ini_pos[] = {
    posInit[indexPoint[0]],
    posInit[indexPoint[1]]
  };
  math::vec3 cur_CM = (cur_pos[0] * mass[0] + cur_pos[1] * mass[1]) / (mass[0] + mass[1]);
  math::vec3 ini_CM = (ini_pos[0] * mass[0] + ini_pos[1] * mass[1]) / (mass[0] + mass[1]);

  math::vec3 axis = (cur_pos[1] - cur_pos[0]).normalized();
  math::vec3 goal[] = {
    cur_CM - (axis * (ini_pos[0] - ini_CM).length()),
    cur_CM + (axis * (ini_pos[1] - ini_CM).length())
  };

  math::vec3 offset[] = {
    (goal[0] - cur_pos[0]) * invNbrAdjEdges[indexPoint[0]] * edgeStiffness,
    (goal[1] - cur_pos[1]) * invNbrAdjEdges[indexPoint[1]] * edgeStiffness
  };

  for (size_t i = 0; i < 2; i++)
  {
#ifdef UPDATE_ALL_AT_ONCE
#ifdef USE_IMPULSE
    correction[indexPoint[i]] += offset[i] * INV_PHYSICS_TIME_STEP * mass[indexPoint[i]];
#else
    correction[indexPoint[i]] += offset[i];
#endif
#else
#ifdef USE_IMPULSE
    math::vec3 I = offset[i] * INV_PHYSICS_TIME_STEP * mass[indexPoint[i]];
    _lms.apply_impulse(posCur[indexPoint[i]], I);
#else
    _lms.move_linear_position(posCur[indexPoint[i]], offset[i]);
#endif
#endif
  }
}
void Cloth::update()
{
#ifdef UPDATE_ALL_AT_ONCE
  memset(correction, 0, nbrOfPoints * sizeof(math::vec3));
#endif
  // weight forces
  for (size_t n = 0; n < nbrOfPoints; n++)
  {
    _lms.add_force(posCur[n], math::vec3(0.f, -9.81f, 0.f));
  }

  //size_t fixed[]{ 0, nbrOfPoints - 1, width - 1, nbrOfPoints - width, -1 };
  size_t fixed[]{ 0, nbrOfPoints - 1, -1 };
  for (size_t i = 0; fixed[i] != -1; i++)
  {
    size_t a = fixed[i];
#ifdef USE_IMPULSE_TO_FIX_POINTS
    math::vec3 Ia = math::vec3(posInit[a] - _lms.get_linear_position(posCur[a])) * INV_PHYSICS_TIME_STEP * mass[a];
    _lms.apply_impulse(posCur[a], Ia);
#else
    _lms.move_linear_position(posCur[a], (posInit[a] - _lms.get_linear_position(posCur[a])) * PHYSICS_DAMPING_FACTOR);
#endif
  }

  for (size_t n = 0; n < nbrOfTriangles; n++)
  {
    triangle2DCorrection(n);
    //applyTriangleShapeMatching(n);
  }
  for (size_t n = 0; n < nbrOfEdges; n++)
  {
    //edgeCorrection(n); // use 2D rotation instead?
  }
#ifdef UPDATE_ALL_AT_ONCE
  for (size_t n = 0; n < nbrOfPoints; n++)
  {
#ifdef USE_IMPULSE
    _lms.apply_impulse(posCur[n], correction[n]);
#else
    _lms.move_linear_position(posCur[n], correction[n]);
#endif
  }
#endif
}

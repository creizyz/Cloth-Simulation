#pragma once

#include <vector>
#include "maths\math.h"
#include "graphics\openGL.h"

typedef math::vec3 vector3;
typedef math::mat4 matrix4x4;

struct Program
{
  unsigned int handle;
  unsigned int mvpLocation;
  unsigned int colorLocation;

  unsigned int positionLocation;
};

struct Component
{ };

struct MaterialData
{
  Program program;
  vector3 color;
};

struct VertexDataChunk
{
  size_t count;
  vector3 * position;
  matrix4x4 modelMatrix;

  VertexDataChunk(size_t capacity)
  {
    this->position = (vector3*) malloc(sizeof(vector3) * capacity);
    this->modelMatrix = matrix4x4();
  }

  void release()
  {
    if (this->position != nullptr) free(this->position);
  }
};
struct IndexDataChunk
{
  size_t count;
  size_t * index;

  IndexDataChunk(size_t capacity)
  {
    this->index = (size_t*)malloc(sizeof(size_t) * capacity);
  }

  void release()
  {
    if (this->index != nullptr) free(this->index);
  }
};
typedef IndexDataChunk TriangleDataChunk;
typedef IndexDataChunk EdgeDataChunk;
struct PhysicDataChunk : VertexDataChunk
{
  vector3 * velocity;
  vector3 * force;
  float * mass_inv;
  
  PhysicDataChunk(size_t capacity) : VertexDataChunk(capacity)
  {
    this->velocity = (vector3*)malloc(sizeof(vector3) * capacity);
    this->force    = (vector3*)malloc(sizeof(vector3) * capacity);
    this->mass_inv = (float*)malloc(sizeof(float) * capacity);
  }

  void release()
  {
    if (this->position != nullptr) free(this->position);
    if (this->velocity != nullptr) free(this->velocity);
    if (this->force != nullptr) free(this->force);
    if (this->mass_inv != nullptr) free(this->mass_inv);
  }
};

struct GraphicMeshData : Component
{
  VertexDataChunk vertices;
  TriangleDataChunk triangles;
  EdgeDataChunk edges;
  MaterialData material;

  unsigned int vao;
  unsigned int vbo;
  unsigned int ibo_t;
  unsigned int ibo_e;

  bool valid;

  void init()
  {
    gl::glGenVertexArrays(1, &(this->vao));
    gl::glGenBuffers(1, &(this->vbo));
    gl::glGenBuffers(1, &(this->ibo_t));
    gl::glGenBuffers(1, &(this->ibo_e));

    this->valid = this->vao != 0 && this->vbo != 0 && (this->ibo_t != 0 || this->ibo_e != 0);

    if (this->valid)
    {
      gl::glBindVertexArray(this->vao);
      gl::glBindBuffer(gl::GL_ARRAY_BUFFER, this->vbo);
      gl::glVertexAttribPointer(this->material.program.positionLocation, 3, gl::GL_FLOAT, gl::GL_FALSE, 0, 0);
      if (this->triangles.count > 0)
      {
        gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, this->ibo_t);
        gl::glBufferData(gl::GL_ELEMENT_ARRAY_BUFFER, sizeof(size_t) * 3 * this->triangles.count, this->triangles.index, gl::GL_STATIC_DRAW);
      }
      if (this->edges.count > 0)
      {
        gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, this->ibo_e);
        gl::glBufferData(gl::GL_ELEMENT_ARRAY_BUFFER, sizeof(size_t) * 2 * this->edges.count, this->edges.index, gl::GL_STATIC_DRAW);
      }
      gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, 0);
      gl::glBindBuffer(gl::GL_ARRAY_BUFFER, 0);
      gl::glBindVertexArray(0);
    }
    else
    {
      this->release();
    }
  }
  void release()
  {
    if (this->vao != 0) gl::glDeleteVertexArrays(1, &(this->vao));
    if (this->vbo != 0) gl::glDeleteBuffers(1, &(this->vbo));
    if (this->ibo_t != 0) gl::glDeleteBuffers(1, &(this->ibo_t));
    if (this->ibo_e != 0) gl::glDeleteBuffers(1, &(this->ibo_e));
    this->valid = false;
  }
};



struct ClothMeshData : Component
{
  PhysicDataChunk vertices;
  TriangleDataChunk triangles;
};

template <class T>
class System
{
protected:
  std::vector<T*> objects;
public:
  virtual void add(T * object)
  {
    objects.push_back(object);
  }
  virtual bool remove(T * object)
  {
    for (std::vector<T>::iterator it = objects.begin(); it != objects.end()); ++it)
    {
      if (*it == object) objects.erase(it);
    }
  }
  virtual void update();
};

class GravitySystem : System<PhysicDataChunk>
{
private:
  const vector3 gravity;
public:
  GravitySystem(const vector3 force)
    : gravity(force)
  { }

  void update()
  {
    for (std::vector<PhysicDataChunk*>::iterator it = objects.begin(); it != objects.end(); ++it)
    {
      PhysicDataChunk * data = *it;
      for (size_t n = 0; n < data->count; n++)
      {
        data->force[n] += gravity;
      }
    }
  }
};

struct GameObject
{
  matrix4x4 matrix;
  std::vector<Component> components;
};

class Camera : GameObject
{
private:
  matrix4x4 viewMatrix;
  matrix4x4 projectionMatrix;

  float near;
  float far;
  float fov;
  float aspect;

  float orientX;
  float orientY;

public:
  matrix4x4 getViewMatrix()
  {
    return this->viewMatrix;
  }

  matrix4x4 getProjectionMatrix()
  {
    return this->projectionMatrix;
  }

  void setNearAndFarPlane(float near, float far)
  {
    this->near = (near > 0.1f) ? near : 0.1f;
    this->far = (far < this->near + 0.1f) ? far : 0.1f;
  }

  void setFOV(float fov)
  {
    this->fov = (fov > 0.03f) ? ((fov < 2.2f) ? fov : 2.2f) : 0.03f;
  }

  float getFOV()
  {
    return this->fov;
  }

  float getNear()
  {
    return this->near;
  }

  float getFar()
  {
    return this->far;
  }

  void setAspect(float aspect)
  {
    this->aspect = (aspect > 0.001f) ? aspect : 0.001f;
  }

  float getAspect()
  {
    return this->aspect;
  }

  void translate(const vector3 & t)
  {
    this->pos += t;
  }

  void rotateOnXY(float aroundX, float aroundY)
  {
    orientX += aroundX;
    orientY += aroundY;
  }

  void look_at(const math::vec3 & point)
  {
    if (point != pos)
    {
      math::vec3 dir = (point - pos).normalized();
      orientX = asin(-dir.y);
      orientY = atan2f(-dir.x, -dir.z);
    }
  }

private:
  void normalize()
  {
    while (orientX > 360) orientX -= 360;
    while (orientX < 0) orientX += 360;
    while (orientY > 360) orientY -= 360;
    while (orientY < 0) orientY += 360;
  }
  math::mat4 matrix()
  {
    return projection() * view();
  }
  math::mat4 view()
  {
    return orientation() * math::mat4::T_Transform(-pos);
  }
  math::mat4 projection()
  {
    DBG_VALID_FLOAT(fov);
    DBG_VALID_FLOAT(viewportAspect);
    DBG_VALID_FLOAT(nearPlane);
    DBG_VALID_FLOAT(farPlane);

    math::mat4 projection_matrix(1);
    if (fov > 0 && viewportAspect > 0 && nearPlane != farPlane)
    {
      float tanHalfFov = tan(0.5f * fov);

      projection_matrix.data[0] = 1.f / (tanHalfFov * viewportAspect);
      projection_matrix.data[5] = 1.f / tanHalfFov;
      projection_matrix.data[10] = -(farPlane + nearPlane) / (farPlane - nearPlane);
      projection_matrix.data[11] = -(2.f * farPlane * nearPlane) / (farPlane - nearPlane);
      projection_matrix.data[14] = -1.f;
      projection_matrix.data[15] = 0.f;
    }
    DBG_VALID_MAT4(projection_matrix);
    return projection_matrix;
  }
  math::mat4 orientation()
  {
    return math::mat4::R_Transform(orientX, math::vec3(1.f, 0.f, 0.f)) * math::mat4::R_Transform(orientY, math::vec3(0.f, 1.f, 0.f));
  }
private:
  math::vec3 pos;
  float orientX;
  float orientY;

  float fov;
  float nearPlane;
  float farPlane;
  float viewportAspect;

  void normalize_angles()
  {
    orientY = fmod(orientY, 360.f);
    //if (orientY < 0.f)  orientY += 360.f;
    orientX = std::min(MaxVerticalAngle, std::max(-MaxVerticalAngle, orientX));
  }
};

class RenderingSystem : System<GraphicMeshData>
{
private:
  Camera camera;
public:

  void setCamera(const Camera & camera)
  {
    this->camera = camera;
  }

  void update()
  {
    for (std::vector<GraphicMeshData*>::iterator it = objects.begin(); it != objects.end(); ++it)
    {
      GraphicMeshData * mesh = *it;
      if (mesh->valid)
      {
        gl::glUseProgram(mesh->material.program.handle);
        gl::glUniformMatrix4fv(mesh->material.program.mvpLocation, 16, false, (mesh->vertices.modelMatrix * this->camera.getViewMatrix() * this->camera.getProjectionMatrix()).data);
        gl::glUniform3f(mesh->material.program.colorLocation, mesh->material.color.x, mesh->material.color.y, mesh->material.color.z);

        gl::glBindVertexArray(mesh->vao);
        gl::glBindBuffer(gl::GL_ARRAY_BUFFER, mesh->vbo);
        gl::glBufferData(gl::GL_ARRAY_BUFFER, sizeof(float) * 3 * mesh->vertices.count, mesh->vertices.position, gl::GL_DYNAMIC_DRAW);
        if (mesh->triangles.count > 0)
        {
          gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, mesh->ibo_t);
          gl::glDrawElements(gl::GL_TRIANGLES, 3 * mesh->triangles.count, gl::GL_UNSIGNED_INT, 0);
        }
        if (mesh->edges.count > 0)
        {
          gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, mesh->ibo_e);
          gl::glDrawElements(gl::GL_TRIANGLES, 2 * mesh->edges.count, gl::GL_UNSIGNED_INT, 0);
        }
        gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, 0);
        gl::glBindBuffer(gl::GL_ARRAY_BUFFER, 0);
        gl::glBindVertexArray(0);
      }
    }
  }
};

class ClothSystem : System<ClothMeshData>
{
private:
  std::vector<VertexDataChunk*> initialPositions;

  void ShapeMatching2D(ClothMeshData * mesh, const VertexDataChunk * initialMeshPosisions)
  {
    /*for (size_t i = 0; i < mesh->triangles.count; i++)
    {
      size_t baseIndex = 3 * i;
      size_t indexes[3] = {
        mesh->triangles.index[baseIndex + 0],
        mesh->triangles.index[baseIndex + 1],
        mesh->triangles.index[baseIndex + 2]
      };
      vector3 current[3] = {
        mesh->vertices.position[indexes[0]],
        mesh->vertices.position[indexes[1]],
        mesh->vertices.position[indexes[2]]
      };
      vector3 initial[3] = {
        initialMeshPosisions->position[indexes[0]],
        initialMeshPosisions->position[indexes[1]],
        initialMeshPosisions->position[indexes[2]]
      };
    }*/
  }
  void ShapeMatching1D(ClothMeshData * mesh, const VertexDataChunk * initialMeshPosisions)
  {

  }
public:
  void add(ClothMeshData * cloth) override
  {
    this->objects.push_back(cloth);
    VertexDataChunk * copy; // = allocate(size);
    if (copy->position != nullptr) memcpy(copy->position, cloth->vertices.position, cloth->vertices.count);
    this->initialPositions.push_back(copy);
  }
  bool remove(ClothMeshData * cloth) override
  {
    std::vector<ClothMeshData*>::iterator meshIt = this->objects.begin();
    std::vector<VertexDataChunk*>::iterator initIt = this->initialPositions.begin();
    for (size_t i = 0; i < this->objects.size; i++)
    {
      meshIt++;
      initIt++;
      if (*meshIt == cloth)
      {
        // release(*initIt)
        this->objects.erase(meshIt);
        this->initialPositions.erase(initIt);
        break;
      }
    }
  }
  void update()
  {
    // compute forces
  }
};

class ParticleMotionSystem : System<PhysicDataChunk>
{
private:
  float _dumpFactor;
  float _timeStep;

public:
  void update()
  {
    for (std::vector<PhysicDataChunk*>::iterator it = objects.begin(); it != objects.end(); ++it)
    {
      PhysicDataChunk * data = *it;
      for (size_t n = 0; n < data->count; n++)
      {
        data->velocity[n] = (data->velocity[n] * _dumpFactor) + (data->force[n] * data->mass_inv[n] * _timeStep);
        data->position[n] += data->velocity[n] * _timeStep;
        data->force[n] = vector3();
      }
    }
  }
};

/*
class CollisionSystem : System<CollisionDataChunk>
{
  void update()
  {
    // check collisions and ...
  }
};
*/

class EventHandler
{
public:
  void update(Application & app)
  {

  }
};

class Application
{
private:
  bool running;
  RenderingSystem _renderingSystem;
  ParticleMotionSystem _particleMotionSystem;
  GravitySystem _gravitySystem;
  ClothSystem _clothSystem;
  EventHandler _eventHandler;

  void init();
  void update()
  {
    _eventHandler.update(*this);
    _gravitySystem.update();
    _clothSystem.update();
    _particleMotionSystem.update();
    // collision system update
    _renderingSystem.update();
  }
  void release();
  void run()
  {
    this->init();
    while (running)
    {
      this->update();
    }
    this->release();
  }
};
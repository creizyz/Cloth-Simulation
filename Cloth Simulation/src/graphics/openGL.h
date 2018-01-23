#ifndef _INCLUDES_OPENGL_
#define _INCLUDES_OPENGL_

// Disable warning messages 4251
#pragma warning( disable : 4251 ) // glbinding issue

// Rendering Library
#pragma comment(lib, "OpenGL32.Lib")   // glVertex3, glClear, ..
#ifdef _DEBUG
  #pragma comment(lib, "glbindingd.Lib")
#else
  #pragma comment(lib, "glbinding.Lib")
#endif
#pragma comment(lib, "glfw3.Lib")

#define GLFW_INCLUDE_NONE

#include <glbinding/gl/gl.h>
#include <glbinding/Binding.h>
#include "GLFW\glfw3.h" // GLFW, OpenGL

extern void glfwInitAndCreateWindow(GLFWwindow * & window, int width, int height, std::string title);
extern void glfwErrorCallback(int error, const char* description);
extern void glfwKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

extern void glBindingInit();
extern void afterOpenglCallback(const glbinding::FunctionCall & call);
extern void beforOpenglCallback(const glbinding::FunctionCall & call);

#define OPENGL_LOG_BUFFER_SIZE 512

namespace OpenGL
{
  extern char LogBuffer[OPENGL_LOG_BUFFER_SIZE];

  namespace shader
  {
    extern gl::GLuint compile(const char * src, gl::GLenum type);
    extern gl::GLuint createProgram(gl::GLuint vertex_shader, gl::GLuint fragment_shader);
    extern gl::GLuint createProgram(const char * src_vertex_shader, const char * src_fragment_shader);
  }
}

#endif
#include "openGL.h"
#include "../common/macro.h"
#include <iostream>
#include <fstream>

void glfwInitAndCreateWindow(GLFWwindow * & window, int width, int height, std::string title)
{
  // Try to initialise GLFW
  if (!glfwInit())
    exit(EXIT_FAILURE);
  else
  {
    glfwSetErrorCallback(glfwErrorCallback);

    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, true); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); //We don't want the old OpenGL 

                                                                   // Try to initialise the window
    if (!(window = glfwCreateWindow(width, height, title.c_str(), NULL, NULL)))
    {
      glfwTerminate();
      exit(EXIT_FAILURE);
    }
    else
    {
      // Make the window context the current OpenGL context
      glfwMakeContextCurrent(window);
      // Set the key callback
      //glfwSetKeyCallback(window, glfwKeyCallback);
    }
  }
}
void glfwErrorCallback(int error, const char* description)
{
  fprintf(stderr, "GLFW Error: (%d) %s\n", error, description);
  DBG_HALT;
}

void glBindingInit()
{
  glbinding::Binding::initialize();
  glbinding::setCallbackMaskExcept(glbinding::CallbackMask::After, { "glGetError" });
  glbinding::setAfterCallback(afterOpenglCallback);
#ifdef ENABLE_OPENGL_LOGS
  glbinding::setCallbackMask(glbinding::CallbackMask::Before | glbinding::CallbackMask::ParametersAndReturnValue);
  glbinding::setBeforeCallback(beforOpenglCallback);
#endif

  gl::glEnable(gl::GL_PROGRAM_POINT_SIZE);
}
void beforOpenglCallback(const glbinding::FunctionCall & call) {
  std::cout << call.function->name() << std::endl;
}
void afterOpenglCallback(const glbinding::FunctionCall & call) {
  gl::GLenum error = gl::glGetError();
  if (error != gl::GL_NO_ERROR)
  {
#ifndef ENABLE_OPENGL_LOGS
    std::cout << call.function->name() << std::endl;
#endif
    std::cout << "<> error: " << std::hex << error << std::endl;
    DBG_HALT;
  }
}

gl::GLuint compile_shader(const char * shaderSource, gl::GLenum shaderType, bool inFile)
{
  const char * shaderSourceCode(nullptr);
  std::string text;

  if (!inFile)
  {
    shaderSourceCode = shaderSource;
  }
  else
  {
    std::ifstream ifs;
    ifs.open(shaderSource, std::ifstream::in);
    if (!ifs.is_open())
    {
      // TODO: LOG
      DBG_HALT;
    }
    else
    {
      std::getline(ifs, text, (char)EOF);
      ifs.close();
    }
    shaderSourceCode = text.c_str();
  }

  // Compile the shader
  gl::GLuint shader = gl::glCreateShader(shaderType);
  gl::glShaderSource(shader, 1, &shaderSourceCode, NULL);
  gl::glCompileShader(shader);

  // Check the compilation result
  gl::GLint test;
  gl::glGetShaderiv(shader, gl::GL_COMPILE_STATUS, &test);
  if (!test)
  {
    // TODO: LOG
    std::cerr << "Shader compilation failed with this message:" << std::endl;
    std::vector<char> compilation_log(512);
    gl::glGetShaderInfoLog(shader, compilation_log.size(), NULL, &compilation_log[0]);
    std::cerr << &compilation_log[0] << std::endl;
    glfwTerminate();
    DBG_HALT;
    exit(EXIT_FAILURE);
  }

  return shader;
}

char OpenGL::LogBuffer[OPENGL_LOG_BUFFER_SIZE];

gl::GLuint OpenGL::shader::compile(const char * src, gl::GLenum type)
{
  gl::GLint status;
  gl::GLuint shader = gl::glCreateShader(type);
  if (shader != 0)
  {
    gl::glShaderSource(shader, 1, &src, NULL);
    gl::glCompileShader(shader);
    gl::glGetShaderiv(shader, gl::GL_COMPILE_STATUS, &status);
    if (status == (gl::GLint)gl::GL_FALSE)
    {
      gl::glGetShaderInfoLog(shader, OPENGL_LOG_BUFFER_SIZE, NULL, OpenGL::LogBuffer);
      std::cout << "shader compilation failed : " << OpenGL::LogBuffer << std::endl;
      gl::glDeleteShader(shader);
      shader = 0;
    }
  } 
  return shader;
}
gl::GLuint OpenGL::shader::createProgram(gl::GLuint vertex_shader, gl::GLuint fragment_shader)
{
  if (vertex_shader == 0 || vertex_shader == 0)
  {
    std::cout << "program creation failed : input shader is not valid (input handle == 0)" << std::endl;
    return 0;
  }

  gl::GLint status;
  gl::GLuint program = gl::glCreateProgram();
  if (program != 0)
  {
    gl::glAttachShader(program, vertex_shader);
    gl::glAttachShader(program, fragment_shader);
    gl::glLinkProgram(program);
  }
  return program;
}

gl::GLuint OpenGL::shader::createProgram(const char * src_vertex_shader, const char * src_fragment_shader)
{
  gl::GLuint vertex_shader = compile(src_vertex_shader, gl::GL_VERTEX_SHADER);
  gl::GLuint fragment_shader = compile(src_fragment_shader, gl::GL_FRAGMENT_SHADER);
  gl::GLuint program = createProgram(vertex_shader, fragment_shader);
  if (vertex_shader != 0)
  {
    gl::glDeleteShader(vertex_shader);
  }
  if (fragment_shader != 0)
  {
    gl::glDeleteShader(fragment_shader);
  }
  return program;
}
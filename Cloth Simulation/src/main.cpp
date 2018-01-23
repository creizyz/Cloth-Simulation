#include "./graphics/openGL.h"
#include "./graphics/CameraController.h"
#include "./physics/cloth.h"
#include "./physics/BVH.h"
#include "./common/Utils.h"
#include "./graphics/Grid.h"

GLFWwindow * window;
gl::GLuint uniformColorProgram;
gl::GLuint vertexColorProgram;
gl::GLuint strainColorProgram;

Camera3D camera;
CameraController camera_control(camera);

std::chrono::high_resolution_clock::time_point last_update;
float update_time_correction = 0.f;

Grid groundGrid;

#define CLOTH_RESOLUTION 6
#define CLOTH_EDGE_SIZE .1f
#define CLOTH_THICKNESS .05f

#define COLLISION

LinearMotionSystem lms(10000);
Cloth cloth(lms, math::vec3(0.f, 1.f, 0.f), math::vec3(0.f, 0.f, 1.f), math::vec3(1.f, 0.f, 0.f), CLOTH_RESOLUTION, CLOTH_RESOLUTION, CLOTH_EDGE_SIZE, CLOTH_EDGE_SIZE);
#ifdef COLLISION
ClothCollisionModel clothCollisionModel(lms);
#endif

double timerClothUpdate(0);
double minTm(1000000);
double maxTm(0);
int nbrOfFrames(0);

#define VERTEX_SHADER_COUNT 3
#define FRAGMENT_SHADER_COUNT 1

void initShaders()
{
  std::string vertexShaderSrc[VERTEX_SHADER_COUNT];
  vertexShaderSrc[0] = utils::readFile("./shader/basic.vert");
  vertexShaderSrc[1] = utils::readFile("./shader/uniformColor.vert");
  vertexShaderSrc[2] = utils::readFile("./shader/strainColor.vert");

  std::string fragmentShaderSrc[FRAGMENT_SHADER_COUNT];
  fragmentShaderSrc[0] = utils::readFile("./shader/basic.frag");

  gl::GLuint vertexShader[VERTEX_SHADER_COUNT];
  for (int i = 0; i < VERTEX_SHADER_COUNT; i++) vertexShader[i] = OpenGL::shader::compile(vertexShaderSrc[i].c_str(), gl::GL_VERTEX_SHADER);

  gl::GLuint fragmentShader[FRAGMENT_SHADER_COUNT];
  for (int i = 0; i < FRAGMENT_SHADER_COUNT; i++) fragmentShader[i] = OpenGL::shader::compile(fragmentShaderSrc[i].c_str(), gl::GL_FRAGMENT_SHADER);

  uniformColorProgram = OpenGL::shader::createProgram(vertexShader[0], fragmentShader[0]);
  vertexColorProgram = OpenGL::shader::createProgram(vertexShader[1], fragmentShader[0]);
  strainColorProgram = OpenGL::shader::createProgram(vertexShader[2], fragmentShader[0]);

  if (uniformColorProgram + vertexColorProgram + strainColorProgram == 0)
  {
    std::cout << "no shader available, shutting down" << std::endl;
    exit(EXIT_FAILURE);
  }
}

void init()
{
  std::cout << "Init start..." << std::endl;
  glfwInitAndCreateWindow(window, 800, 600, "ClothSimulation");
  glBindingInit();

  gl::glEnable(gl::GL_DEPTH_TEST);
  gl::glEnable(gl::GL_BLEND);
  gl::glBlendFunc(gl::GL_SRC_ALPHA, gl::GL_ONE_MINUS_SRC_ALPHA);

  initShaders();

  int w_width, w_height;
  glfwGetWindowSize(window, &w_width, &w_height);
  camera_control.init_camera(1.f, ((float)w_width / (float)w_height), 1.f, 1000.f);
  camera_control.init_speeds(2.f, 1.57f, 1.f);
  camera.translate(math::vec3(CLOTH_RESOLUTION * .5f * CLOTH_EDGE_SIZE, .8f, 2.f));

  groundGrid.init(uniformColorProgram, math::vec3(0.f, 0.f, 0.f), math::vec3(1.f, 0.f, 0.f), math::vec3(0.f, 0.f, 1.f), 1.f, 50, .5f, .5f, 1.f, .1f, true);

  cloth.setColor(1.f, 1.f, 1.f, .5f);
  cloth.setDensity(1.f);
  cloth.setStiffness(1.f, 1.f);
  cloth.setThickness(CLOTH_THICKNESS);
  cloth.initGL(uniformColorProgram, strainColorProgram);

#ifdef COLLISION
  clothCollisionModel.init(cloth);
  clothCollisionModel.initGL(uniformColorProgram);
  clothCollisionModel.setStiffess(1.f);
#endif

  last_update = std::chrono::high_resolution_clock::now();
  std::cout << "Init end..." << std::endl;
}
void handleEvents()
{
  std::cout << "handle events start..." << std::endl;
  if (glfwGetKey(window, GLFW_KEY_ESCAPE)) glfwSetWindowShouldClose(window, true);

  for (int action = 0; action < COUNT_CAMERA_ACTIONS; action++)
  {
    auto keystatus = glfwGetKey(window, camera_control.actionkey[action]);
    if (keystatus == GLFW_PRESS)   camera_control.log_action((CAMERA_ACTION)action);
    else if (keystatus == GLFW_RELEASE) camera_control.unlog_action((CAMERA_ACTION)action);
  }
  std::cout << "handle events end..." << std::endl;
}
void update()
{
  std::cout << "Update start..." << std::endl;
  std::chrono::high_resolution_clock::time_point beginning_current_update = std::chrono::high_resolution_clock::now();
  float elapsed_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(beginning_current_update - last_update).count() * (float) 1e-9 + update_time_correction;
  int nbrOfUpdates = elapsed_seconds / PHYSICS_TIME_STEP;

  double tmClothUpdate(0.f);
  double tmClothCollision(0.f);
  for (int n = 0; n < nbrOfUpdates; n++)
  {
    lms.update_data();
    auto t1 = std::chrono::high_resolution_clock::now();
    cloth.update();
    double tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - t1).count() * 1e-9;
    //tmClothUpdate += tmp / (double) nbrOfUpdates;
    timerClothUpdate += tmp;
    nbrOfFrames++;

    if (tmp > maxTm) maxTm = tmp;
    else if (tmp < minTm) minTm = tmp;
#ifdef COLLISION
    auto t2 = std::chrono::high_resolution_clock::now();
    clothCollisionModel.resolveInternalCollisions();
    tmClothCollision += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - t2).count() * 1e-9 / (double)nbrOfUpdates;
#endif COLLISION
  }
  update_time_correction = (elapsed_seconds - (nbrOfUpdates * PHYSICS_TIME_STEP));

  //timerClothUpdate += tmClothUpdate;
  //nbrOfFrames++;

  std::cout << timerClothUpdate / (double) nbrOfFrames << " over " << nbrOfFrames << " (min: " << minTm << " - max: " << maxTm << " )" << std::endl;

  camera_control.update_camera(elapsed_seconds);

  last_update = beginning_current_update;
  std::cout << "Update end..." << std::endl;
}
void render()
{
  std::cout << "Render start..." << std::endl;
  math::mat4 projMatrix = camera.matrix();
  gl::glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  gl::glClear(gl::GL_COLOR_BUFFER_BIT | gl::GL_DEPTH_BUFFER_BIT);

  groundGrid.render(projMatrix);
  cloth.render(projMatrix);

#ifdef COLLISION
  clothCollisionModel.render(projMatrix);
#endif

  glfwSwapBuffers(window);
  std::cout << "Render end..." << std::endl;
}

int main(int argc, char * argv[])
{
  init();
  while (!glfwWindowShouldClose(window))
  {
    handleEvents();
    update();
    render();
    glfwPollEvents();
    std::string name;
    std::cout << "next frame? ";
    std::getline(std::cin, name);
  }
  gl::glDeleteProgram(uniformColorProgram);
  gl::glDeleteProgram(vertexColorProgram);
  gl::glDeleteProgram(strainColorProgram);
  return 0; 
}
// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>
#include <iostream>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void pendulumController(const mjModel* m, mjData* d) {
  mj_energyPos(m, d);
  mj_energyVel(m, d);

  // potential energy
  std::cout << d->time << " potential energy: " << d->energy[0] << " kinetic energy: " << d->energy[1] << " total: " << d->energy[0] + d->energy[1] << std::endl;

  // M * qacc + qfrc_bias = qfrc_applied + ctrl
  // M * qddot + f =  qfrc_applied + ctrl
  const int nv = 2; // 2 degrees of freedom
  // initialize the array for mass calculation
  double dense_M[nv * nv] = {0};
  // do the calculation - Convert sparse inertia matrix M into full (i.e. dense) matrix. -> https://mujoco.readthedocs.io/en/latest/APIreference.html#mj-fullm
  mj_fullM(m, dense_M, d->qM);

  double M[nv][nv] = {0};
  M[0][0] = dense_M[0];
  M[0][1] = dense_M[1];
  M[1][0] = dense_M[2];
  M[1][1] = dense_M[3];
  std::cout << "M matrix: " << std::endl;
  std::cout << M[0][0] << "\t" << M[0][1] << std::endl;
  std::cout << M[1][0] << "\t" << M[1][1] << std::endl;

  double qddot[nv] = {0};
  // get acceleration for each DoF -> https://mujoco.readthedocs.io/en/latest/APIreference.html#mjdata
  qddot[0] = d->qacc[0];
  qddot[1] = d->qacc[1];

  double f[nv] = {d->qfrc_bias[0], d->qfrc_bias[1]};

  double lhs[nv] = {0};
  mju_mulMatVec(lhs, dense_M, qddot, 2, 2); // lhs = M * qddot;
  // lhs = M * qddot + f
  lhs[0] = lhs[0] + f[0];
  lhs[1] = lhs[1] + f[1];

  // M * qddot + f =  qfrc_applied + ctrl
  // this will apply force to neutralize the gravity and coriolis(?) so the pendulum will not move (since the ctrl is 0)
  d->qfrc_applied[0] = f[0];
  d->qfrc_applied[1] = f[1];

  // control
  // move from the initial position of 0.5 to -0.5
  double Kp1, Kp2 = 100;
  double Kv1, Kv2 = 10;
  double qref1 = -0.5, qref2 = -1.6;

  // coriolis + gravity + PD Control - for some reason, this doesn't work
//  d->qfrc_applied[0] = f[0] - Kp1 * (d->qpos[0] - qref1) - Kv1 * d->qvel[0];
//  d->qfrc_applied[1] = f[1] - Kp2 * (d->qpos[1] - qref2) - Kv2 * d->qvel[1];


}

// main function
int main(int argc, const char** argv) {
  // check command-line arguments
  if (argc!=2) {
    std::printf(" USAGE:  basic modelfile\n");
    return 0;
  }

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (std::strlen(argv[1])>4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb")) {
    m = mj_loadModel(argv[1], 0);
  } else {
    m = mj_loadXML(argv[1], 0, error, 1000);
  }
  if (!m) {
    mju_error_s("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  cam.azimuth = 90;
  cam.elevation = -20;
  cam.distance = 5;
  cam.lookat[2] = 2;

  mjcb_control = pendulumController;

  d->qpos[0] = 0.5;

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0/60.0) {
      mj_step(m, d);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}

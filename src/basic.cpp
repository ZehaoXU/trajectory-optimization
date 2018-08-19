#include "mujoco.h"
#include "glfw3.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <fstream>
#include <sstream>

using namespace std;

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

int pointDimension;
int positionDimension;
int numberOfPoints;
double timeStep;
ifstream dataFile;
vector<double> trajectory;

int timeIndex = 0;

// initialize trajectroy
void initializeTrajectory(string dir)
{
    dataFile.open(dir.c_str());
    string line;
    string label;
    int count = 0;

    while (!dataFile.eof())
    {
        getline(dataFile, line);

        stringstream ss(line);
        ss >> label;

        if (label.compare("timeStep:") == 0)
        {
            double val;
            ss >> val;
            timeStep = val;
            continue;
        }
        if (label.compare("pointDimension:") == 0)
        {
            int val;
            ss >> val;
            pointDimension = val;
            positionDimension = pointDimension / 3;
            continue;
        }
        if (label.compare("numberOfPoints:") == 0)
        {
            int val;
            ss >> val;
            numberOfPoints = val;
            continue;
        }
        if (label.compare(to_string(count)) == 0)
        {
            count ++;
            double val;
            ss >> val;
            trajectory.push_back(val);
            continue;
        }
    }
}

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

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
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

// change position
void setPosition(int index)
{
    for (int i = 0; i < positionDimension; i++)
        d->qpos[i] = trajectory[index * pointDimension + i];
    mju_zero(d->qvel, positionDimension);
    mju_zero(d->ctrl, positionDimension);
    mju_zero(d->qfrc_applied, m->nv);
}

// main function
int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=2 )
    {
        printf(" USAGE:  basic modelfile\n");
        return 0;
    }

    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";
    if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
        m = mj_loadModel(argv[1], 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);
    mjv_makeScene(&scn, 1000);                   // space for 1000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_100);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    string data_dir = "../log/data1.txt";
    initializeTrajectory(data_dir);
    cout << timeStep << "  " << pointDimension << "  " << numberOfPoints << "  " << trajectory.size() << endl;

    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        
        if (timeIndex < numberOfPoints - 1)
        {
            int oldIndex = timeIndex;
            timeIndex = (int) d->time - 1;
            if ((oldIndex != timeIndex) && (timeIndex >= 0))
                setPosition(timeIndex);
        }
        cout << d->time << endl;
        mj_step(m,d);

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

    // close GLFW, free visualization storage
    glfwTerminate();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    return 1;
}
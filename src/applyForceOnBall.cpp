//---------------------------------//
//  This file is part of MuJoCo    //
//  Written by Emo Todorov         //
//  Copyright (C) 2017 Roboti LLC  //
//---------------------------------//


#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <iostream>


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

// apply force on the ball by set d -> ctrl
void applyForceByModifyDctrl(const mjModel* m, mjData* d)
{   
    // 1. modify d -> ctrl
    if (d -> time > 20 && d -> time < 30)
        d -> ctrl[1] = {-40.0};
    if (d -> time >=30 && d -> time < 50)
        d -> ctrl[1] = {40.0};
    if (d -> time >= 50 && d -> time <60)
        d -> ctrl[1] = {-40.0};
     if (d -> time >= 60)
        d -> ctrl[1] = {0};
}

void applyForceByModifyDqfrc_applied(const mjModel* m, mjData* d, double force, int direction)
{
    // 2. modify d -> qfrc_applied, nv * 1
    if (d -> time > 20 && d -> time < 30)
        d -> qfrc_applied[direction] = {-1 * force};
    if (d -> time >=30 && d -> time < 50)
        d -> qfrc_applied[direction] = {0};
    // if (d -> time >= 50 && d -> time <60)
    //     d -> qfrc_applied[direction] = {-1 * force};
    // if (d -> time >= 60)
    //     d -> qfrc_applied[direction] = {0};
}

void applyForceByModifyDxfrc_applied(const mjModel* m, mjData* d)
{
    // 3. modify d -> xfrc_applied, nbody * 6
    // xfrc_applied: 6 - left, 7 - up, 9 - down, 10 - right
    if (d -> time > 20 && d -> time < 30)
        d -> xfrc_applied[7] = {-40.0};
    if (d -> time >=30 && d -> time < 50)
        d -> xfrc_applied[7] = {40.0};
    if (d -> time >= 50 && d -> time <60)
        d -> xfrc_applied[7] = {-40.0};
    if (d -> time >= 60)
        d -> xfrc_applied[7] = {0};
}

// using mj_applyFT function 
bool flag1 = 0, flag2 = 0, flag3 = 0;
void applyForceByMj_applyFT(const mjModel* m, mjData* d)
{
    mjtNum force[3] = {};
    
    if ((d -> time > 20) && (flag1 == 0))
    {
        force[1] = -100;
        flag1 = 1;
    }
    else if (d -> time > 30 && flag2 == 0)
    {
        force[1] = 200;
        flag2 = 1;
    }
    else if (d -> time > 50 && flag3 == 0)
    {
        force[1] = -200;
        flag3 = 1;
    }
    else
        force[1] = 0;
    if (d -> time > 60)
        d->qfrc_applied[1] = 0;
    mjtNum torque[3] = {};
    mjtNum point[3] = {0,0,0};
    int body = 1;
    std::cout << "force: " << force[0] << std::endl;
    mju_zero(d->qfrc_applied + 3, 3);
    mj_applyFT(m, d, force, torque, point, body, d->qfrc_applied);
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

    // set mass
    mj_setTotalmass(m, 100);

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

    // print some arguments
    // m -> nu = 2;
    // std::cout << m -> nu << std::endl;
    // std::cout << m -> nv << std::endl;
    // std::cout << d -> ctrl << std::endl;

    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        
        while( d->time - simstart < 1.0/60.0)
        {
            mj_step1(m, d);
            applyForceByModifyDqfrc_applied(m, d, 10, 2);
            std::cout << d->qacc[0] << "  " << d->qacc[1] << "  " << d->qacc[2] << std::endl;
            std::cout << d -> time << std::endl;
            mj_step2(m, d);
            // applyForceByMj_applyFT(m, d);
            // std::cout << d->qfrc_applied[0] << "  " << d->qfrc_applied[1] << "  " << d->qfrc_applied[2] << "  "
            //     << d->qfrc_applied[3] << "  " << d->qfrc_applied[4] << "  " << d->qfrc_applied[5] << std::endl;
            // std::cout << d->time << std::endl;
            // mj_step(m, d);
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

// #include <gtest/gtest.h> 
// #include <gmock/gmock.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include "mujoco.h"
#include "trajectoryOptimization/mujocoDynamic.hpp"



// using namespace testing;
using namespace trajectoryOptimization::mujocoDynamic;
using namespace std;    // take care

mjModel* m = NULL;
mjData* d = NULL;

int main(int argc, const char ** argv)
{
    if (argc != 2)
    {
        cout << "ERROR: need model file!" << endl;
        return 0;
    }
    
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "could not load binary model"; // string, memory leak
    if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
        m = mj_loadModel(argv[1], 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // set mass = 10
    mj_setTotalmass(m, 10);

    d = mj_makeData(m);

    // qpos=nq=7, qvel=nv=6, ctrl=nu=0, qfrc_applied=nv=6
    
    // test1: change pos vel, return acc
    dvector position = {1,2,0.2};
    dvector velocity = {2,2,0};
    dvector control = {1,2,98.1};
    const double* acc = getAccUsingMujoco(m, d, position.data(), velocity.data(), control.data());
    cout << "Test1: change pos vel ctrl and return acc" << endl;
    cout << "pos: " << d->qpos[0] << "  " << d->qpos[1] << "  " << d->qpos[2] << endl;
    cout << "vel: " << d->qvel[0] << "  " << d->qvel[1] << "  " << d->qvel[2] << endl;
    cout << "acc: " << d->qacc[0] << "  " << d->qacc[1] << "  " << d->qacc[2] << endl;
    cout << "ctrl:" << d->qfrc_applied[0] << "  "
                    << d->qfrc_applied[1] << "  "
                    << d->qfrc_applied[2] << "  " << endl << endl;

    // test2: step forward, return tuple<vector(position), vector(velocity)>
    cout << "Test2: step forward, return next position and next velocity" << endl;
    
    double dTime = 0.5;
    auto nextTimePoint = stepForward(m, d, position.data(), velocity.data(), control.data(),dTime);
    cout << "next pos: " << get<0>(nextTimePoint)[0] << "  " 
                        << get<0>(nextTimePoint)[1] << "  "
                        << get<0>(nextTimePoint)[2] << endl;
    cout << "next vel: " << get<1>(nextTimePoint)[0] << "  "
                        << get<1>(nextTimePoint)[1] << "  "
                        << get<1>(nextTimePoint)[2] << endl;

    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    return 1;
}
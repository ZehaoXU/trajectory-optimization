#include <gtest/gtest.h> 
#include <gmock/gmock.h>
#include<iostream>
#include "trajectoryOptimization/dynamic.hpp"

using namespace testing;
using namespace trajectoryOptimization::dynamic;

class MujocoTest : public :: testing :: Test
{
public:
    void SetUp() override
    {
        mj_activate("../../mjkey.txt");    
        // load and compile model
        char error[1000] = "ERROR: could not load binary model!";
        _m = mj_loadXML("../../model/ball.xml", 0, error, 1000);
        mj_setTotalmass(_m, 1);
        _d = mj_makeData(_m);
    }

    void TearDown() override
    {
        mj_deleteData(_d);
        mj_deleteModel(_m);
        mj_deactivate();
    }

    mjModel* _m;
    mjData* _d;
};

TEST_F(MujocoTest, controlVelocityZero)
{
    dvector position = {1, 2, 0};
    dvector velocity = {0, 0, 0};
    dvector control = {0, 0, 0};

    auto getNextPosVel = GetNextPositionVelocityUsingMujoco(_m, _d);
    auto [nextPosition, nextVelocity] = getNextPosVel(position.data(),
                                    velocity.data(),
                                    control.data());
    
    EXPECT_THAT(position, nextPosition);
    EXPECT_THAT(velocity, nextVelocity);
}

TEST_F(MujocoTest, controlZero) 
{
    dvector position = {1, 2, 0};
    dvector velocity = {2, 2, 0};
    dvector control = {0, 0, 0};

    auto getNextPosVel = GetNextPositionVelocityUsingMujoco(_m, _d);
    auto [nextPosition, nextVelocity] = getNextPosVel(position.data(),
                                    velocity.data(),
                                    control.data());
    
    EXPECT_NEAR(nextPosition[0], 2, 1e-5);
    EXPECT_NEAR(nextPosition[1], 3, 1e-5);
    EXPECT_NEAR(nextPosition[2], 0, 1e-5);
    EXPECT_THAT(nextVelocity, ElementsAre(2, 2, 0));
}

TEST_F(MujocoTest, controlOne) 
{
    dvector position = {1, 2, 0};
    dvector velocity = {2, 2, 0};
    dvector control = {1, 0, 0};

    auto getNextPosVel = GetNextPositionVelocityUsingMujoco(_m, _d);
    auto [nextPosition, nextVelocity] = getNextPosVel(position.data(),
                                    velocity.data(),
                                    control.data());
    
    EXPECT_NEAR(nextPosition[0], 2.125, 1e-5);
    EXPECT_NEAR(nextPosition[1], 3, 1e-5);
    EXPECT_NEAR(nextPosition[2], 0, 1e-5);
    EXPECT_NEAR(nextVelocity[0], 2.5, 1e-5);
    EXPECT_NEAR(nextVelocity[1], 2, 1e-5);
    EXPECT_NEAR(nextVelocity[2], 0, 1e-5);
}

TEST_F(MujocoTest, controlOneTwo) 
{
    dvector position = {1, 2, 0};
    dvector velocity = {2, 2, 0};
    dvector control = {1, 2, 0};

    auto getNextPosVel = GetNextPositionVelocityUsingMujoco(_m, _d);
    auto [nextPosition, nextVelocity] = getNextPosVel(position.data(),
                                    velocity.data(),
                                    control.data());
    
    EXPECT_NEAR(nextPosition[0], 2.125, 1e-5);
    EXPECT_NEAR(nextPosition[1], 3.25, 1e-5);
    EXPECT_NEAR(nextPosition[2], 0, 1e-5);
    EXPECT_NEAR(nextVelocity[0], 2.5, 1e-5);
    EXPECT_NEAR(nextVelocity[1], 3, 1e-5);
    EXPECT_NEAR(nextVelocity[2], 0, 1e-5);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
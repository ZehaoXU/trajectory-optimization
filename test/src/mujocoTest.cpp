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
    
    EXPECT_THAT(nextPosition, ElementsAre(2, 3, 0));
    EXPECT_THAT(nextVelocity, ElementsAre(2, 2, 0));

}

TEST_F(MujocoTest, controlOne)  // failed!
{
    dvector position = {1, 2, 0};
    dvector velocity = {2, 2, 0};
    dvector control = {1, 0, 0};

    auto getNextPosVel = GetNextPositionVelocityUsingMujoco(_m, _d);
    auto [nextPosition, nextVelocity] = getNextPosVel(position.data(),
                                    velocity.data(),
                                    control.data());
    
    EXPECT_THAT(nextPosition, ElementsAre(2.125, 3, 0));
    EXPECT_THAT(nextVelocity, ElementsAre(2.5, 2, 0));

}


int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
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
        _m->opt.timestep = 0.00001;
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

TEST_F(MujocoTest, 2DControlZero)
{
    dvector position = {1, 2};
    dvector velocity = {0, 0};
    dvector control = {0, 0};
    int worldDimension = 2;

    auto getAcceleration = GetAccelerationUsingMujoco(_m, _d, worldDimension);
    auto acc = getAcceleration(position.data(), velocity.data(), control.data());
    dvector acceleration(acc, acc + worldDimension);

    EXPECT_THAT(acceleration, ElementsAre(0, 0));
}

TEST_F(MujocoTest, 2DControlOne)
{
    dvector position = {1, 2};
    dvector velocity = {0, 0};
    dvector control = {1, 0};
    int worldDimension = 2;

    auto getAcceleration = GetAccelerationUsingMujoco(_m, _d, worldDimension);
    auto acc = getAcceleration(position.data(), velocity.data(), control.data());
    dvector acceleration(acc, acc + worldDimension);

    EXPECT_THAT(acceleration, ElementsAre(1, 0));
}

TEST_F(MujocoTest, 2DControlOneTwo)
{
    dvector position = {1, 2};
    dvector velocity = {0, 0};
    dvector control = {1, 2};
    int worldDimension = 2;

    auto getAcceleration = GetAccelerationUsingMujoco(_m, _d, worldDimension);
    auto acc = getAcceleration(position.data(), velocity.data(), control.data());
    dvector acceleration(acc, acc + worldDimension);

    EXPECT_THAT(acceleration, ElementsAre(1, 2));
}

TEST_F(MujocoTest, controlZero)
{
    dvector position = {1, 2, 0};
    dvector velocity = {0, 0, 0};
    dvector control = {0, 0, 0};

    auto getAcceleration = GetAccelerationUsingMujoco(_m, _d);
    auto acc = getAcceleration(position.data(), velocity.data(), control.data());
    dvector acceleration(acc, acc + 3);

    EXPECT_THAT(acceleration, ElementsAre(0, 0, 0));
}

TEST_F(MujocoTest, controlOne)
{
    dvector position = {1, 2, 0};
    dvector velocity = {0, 0, 0};
    dvector control = {1, 0, 0};

    auto getAcceleration = GetAccelerationUsingMujoco(_m, _d);
    auto acc = getAcceleration(position.data(), velocity.data(), control.data());
    dvector acceleration(acc, acc + 3);

    EXPECT_THAT(acceleration, ElementsAre(1, 0, 0));
}

TEST_F(MujocoTest, controlOneTwo)
{
    dvector position = {1, 2, 0};
    dvector velocity = {0, 0, 0};
    dvector control = {1, 2, 0};

    auto getAcceleration = GetAccelerationUsingMujoco(_m, _d);
    auto acc = getAcceleration(position.data(), velocity.data(), control.data());
    dvector acceleration(acc, acc + 3);

    EXPECT_THAT(acceleration, ElementsAre(1, 2, 0));
}

TEST_F(MujocoTest, controlOneTwoThree)
{
    dvector position = {1, 2, 0};
    dvector velocity = {0, 0, 0};
    dvector control = {1, 2, 3};

    auto getAcceleration = GetAccelerationUsingMujoco(_m, _d);
    auto acc = getAcceleration(position.data(), velocity.data(), control.data());
    dvector acceleration(acc, acc + 3);

    EXPECT_THAT(acceleration, ElementsAre(1, 2, 3));
}

TEST_F(MujocoTest, getPositionControlVelocityZero)
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

TEST_F(MujocoTest, getPositionVelocityControlZero) 
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

TEST_F(MujocoTest, getPositionVelocityControlOne) 
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

TEST_F(MujocoTest, getPositionVelocityControlOneTwo) 
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

TEST_F(MujocoTest, getContactForceNotTouch)
{
    dvector position = {1, 1, 0};
    dvector velocity = {2, 2, 0};
    dvector control = {1, 2, 0};
    
    auto getContactForce = GetContactForceUsingMujoco(_m, _d);
    auto frc = getContactForce(position.data(), velocity.data(), control.data());
    dvector force(frc, frc + 6);
    EXPECT_THAT(force, ElementsAre(0, 0, 0, 0, 0, 0));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
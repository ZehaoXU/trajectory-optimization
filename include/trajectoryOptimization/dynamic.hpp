#pragma once
#include <iostream>
#include <stdio.h>
#include <map>
#include <string>
#include <vector>
#include <cassert>
#include <functional>
#include <range/v3/view.hpp>
#include "mujoco.h"

namespace trajectoryOptimization::dynamic {
	using dvector = std::vector<double>;
	using DynamicFunctionBlock = std::function<const double* (const double*,
													const unsigned,
													const double*,
													const unsigned,
													const double*,
													const unsigned)>;
	using DynamicFunctionMujoco = std::function<const double* (const double*,
													const double*,
													const double*)>;

	using namespace ranges;

	const double* BlockDynamics(const double* position,
							const unsigned positionDimension,
							const double* velocity,
							const unsigned velocityDimension,
							const double* control,
							const unsigned controlDimension){
		assert(positionDimension == velocityDimension);  
		return control;
	}

	std::tuple<dvector, dvector> stepForward(const dvector& position,
											 const dvector& velocity,
											 const dvector& acceleration,
											 const double dt) {
		assert (position.size() == velocity.size()); 
		assert (position.size() == acceleration.size()); 

		const auto moveForwardDt  = [&dt](auto scaler, auto derivative){
			return scaler + derivative*dt;  
		};

		dvector nextPosition = view::zip_with(moveForwardDt, position, velocity); 
		dvector nextVelocity = view::zip_with(moveForwardDt, velocity, acceleration); 
		return {nextPosition, nextVelocity};
	}

	class GetAccelerationUsingMujoco
	{
	  private:
		const double dt;
		const int worldDimension;
		const int controlDimension;
		mjModel* m;
		mjData* d;
	  public:
		GetAccelerationUsingMujoco(const mjModel* _m, mjData* _d, const int dimension, const int controlDimension, const double dTime):
			worldDimension(dimension), dt(dTime),controlDimension(controlDimension)
		{
			m = mj_copyModel(NULL, _m);
			d = mj_copyData(NULL, _m, _d);
		}

		const double* operator() (const double* position, const double* velocity, const double* control)
		{
			// mj_resetData(m, d);
			
			mju_copy(d->qpos, position, worldDimension);
			mju_copy(d->qvel, velocity, worldDimension);
			mju_copy(d->ctrl, control, controlDimension);
			mj_forward(m, d);

			return d->qacc;
		}		
	};

	class GetContactForceUsingMujoco
	{
	  private:
		const double dt;
		const int worldDimension;
		mjModel* m;
		mjData* d;
		double contactForce[12];
	  public:
		GetContactForceUsingMujoco(const mjModel* _m, mjData* _d, const int dimension = 3, const double dTime = 0.5):
			worldDimension(dimension), dt(dTime)
		{
			m = mj_copyModel(NULL, _m);
			d = mj_copyData(NULL, _m, _d);
			memset(contactForce, 0, 12);
		}

		const double* operator() (const double* position, const double* velocity, const double* control)
		{
			// mj_resetData(m, d);
			
			mju_copy(d->qpos, position, worldDimension);
			mju_copy(d->qvel, velocity, worldDimension);
			mju_copy(d->ctrl, control, worldDimension);
			mj_forward(m, d);
			for(int i = 1; i < d->ncon; i ++ )
			mj_contactForce(m, d, i, contactForce + (i - 1) * 6);

			return contactForce;
		}		
	};

	class GetNextPositionVelocityUsingMujoco{
	  private:
		mjModel* m;
		mjData* d;
		const int worldDimension;
		const double dt;
	  public:
		GetNextPositionVelocityUsingMujoco(const mjModel* model, mjData* data, 
										const int dimension = 3, 
										const double dTime = 0.5):
			worldDimension(dimension), dt(dTime)
			{
				m = mj_copyModel(NULL, model);
				d = mj_copyData(NULL, m, data);
			}		

		std::tuple<dvector, dvector> operator()(const double* position,
												const double* velocity,
												const double* control)
		{
			mju_copy(d->qpos, position, worldDimension); 
			mju_copy(d->qvel, velocity, worldDimension);
			mjtNum startTime = d->time;
			while(d->time - startTime < dt)
			{
				mju_copy(d->ctrl, control, worldDimension);
				mj_step(m, d);
			}
			
			dvector nextPosition(d->qpos, d->qpos + worldDimension);
			dvector nextVelocity(d->qvel, d->qvel + worldDimension);

			mj_resetData(m, d);

			return {nextPosition, nextVelocity};
		}

	};

}



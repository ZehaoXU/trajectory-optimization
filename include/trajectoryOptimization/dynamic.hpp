#pragma once
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cassert>
#include <functional>
#include <range/v3/view.hpp>

namespace trajectoryOptimization::dynamic {
	using dvector = std::vector<double>;
	using DynamicFunction = std::function<const double*(const double*,
													const unsigned,
													const double*,
													const unsigned,
													const double*,
													const unsigned)>;
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

	class GetNextPositionVelocityUsingMujoco
	{
	private:
		mjModel* m;
		mjData* d;
		const int worldDimension;
		const double dt;
	public:
		GetNextPositionVelocityUsingMujoco(const mjModel* model, mjData* data, const int dimension, const double dTime):
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
				mj_step1(m, d);
				mju_copy(d->ctrl, control, 3);
				mj_step2(m, d);
			}
			
			dvector nextPosition(d->qpos, d->qpos + 3);
			dvector nextVelocity(d->qvel, d->qvel + 3);

			mj_resetData(m, d);

			return {nextPosition, nextVelocity};
		}
	};

}



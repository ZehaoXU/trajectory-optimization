#pragma once
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <functional>
#include <range/v3/view.hpp>
#include "mujoco.h"

namespace trajectoryOptimization::mujocoDynamic { // dynamic::mujoco, new dynamic test
	using dvector = std::vector<double>;
	using DynamicFunction = std::function<const double*(const double*,
													const unsigned,
													const double*,
													const unsigned,
													const double*,
													const unsigned)>;
	using namespace ranges;
	using namespace std;

	const double* getAccUsingMujoco(const mjModel* m, mjData* d,	// 曲调muj updateMujoco
							const double* position,
							const double* velocity,
							const double* control){
		// set sth
		mju_copy(d->qpos, position, 3);	// 3? const int worldDimension = 3
		mju_copy(d->qvel, velocity, 3);	
		mju_copy(d->qfrc_applied, control, 3);

		// update
		mj_forward(m, d);

		return d->qacc;
	}

	std::tuple<dvector, dvector> stepForward(const mjModel* m, mjData* d,	// get next vel pos
											const double* position,
											const double* velocity,
											const double* control,
											const double dt) { 
		// class including: m, d, dt											
		// local var = const mjData d

		mju_copy(d->qpos, position, 3); // 3?
		mju_copy(d->qvel, velocity, 3);

		mjtNum startTime = d->time;	// dt = mujoco 内部time
		while(d->time - startTime < dt)
		{
			mj_step1(m, d);
			mju_copy(d->qfrc_applied, control, 3);
			mj_step2(m, d);
		}
		// 定义时直接初始化 
		dvector nextPosition, nextVelocity;
		nextPosition.insert(nextPosition.begin(), d->qpos, d->qpos + 3);
		nextVelocity.insert(nextVelocity.begin(), d->qvel, d->qvel + 3);

		return {nextPosition, nextVelocity};	// 考虑vector or array
		// 
		// reset mjData!
	}

}

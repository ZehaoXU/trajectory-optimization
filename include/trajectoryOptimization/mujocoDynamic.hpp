#pragma once
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <functional>
#include <range/v3/view.hpp>
#include "mujoco.h"

namespace trajectoryOptimization::mujocoDynamic {
	using dvector = std::vector<double>;
	using DynamicFunction = std::function<const double*(const double*,
													const unsigned,
													const double*,
													const unsigned,
													const double*,
													const unsigned)>;
	using namespace ranges;
	using namespace std;

	const double* getAccUsingMujoco(const mjModel* m, mjData* d,
							const double* position,
							const double* velocity,
							const double* control){
	
		mju_copy(d->qpos, position, 3);
		mju_copy(d->qvel, velocity, 3);
		mju_copy(d->qfrc_applied, control, 3);
		mj_forward(m, d);

		return d->qacc;
	}

	std::tuple<dvector, dvector> stepForward(const mjModel* m, mjData* d,
											const double* position,
											const double* velocity,
											const double* control,
											const double dt) { 

		dvector nextPosition, nextVelocity;

		mju_copy(d->qpos, position, 3);
		mju_copy(d->qvel, velocity, 3);

		mjtNum startTime = d->time;
		while(d->time - startTime < dt)
		{
			mj_step1(m, d);
			mju_copy(d->qfrc_applied, control, 3);
			mj_step2(m, d);
		}

		nextPosition.insert(nextPosition.begin(), d->qpos, d->qpos + 3);
		nextVelocity.insert(nextVelocity.begin(), d->qvel, d->qvel + 3);

		return {nextPosition, nextVelocity};
	}

}

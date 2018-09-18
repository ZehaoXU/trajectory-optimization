#pragma once
#include <cmath>
#include <cassert>
#include <range/v3/view.hpp> 

#include "utilities.hpp"
#include "dynamic.hpp"
#include "constraint.hpp"


namespace trajectoryOptimization::cost{

	using namespace trajectoryOptimization;
	using namespace trajectoryOptimization::constraint;
	using namespace trajectoryOptimization::dynamic;

	class GetControlSquareSum {
		const unsigned numberOfPoints;
		const unsigned pointDimension;
		const unsigned controlDimension;
		const int trajectoryDimension;
		const int controlStartIndex; 
		const int controlEndIndex;
		const int worldDimension;
		std::vector<unsigned> controlIndices;
		public:
			GetControlSquareSum(const unsigned numberOfPoints,
								const unsigned pointDimension,
								const unsigned controlDimension):
									numberOfPoints(numberOfPoints),
									pointDimension(pointDimension),
									controlDimension(controlDimension),
									trajectoryDimension(numberOfPoints * pointDimension),
									controlStartIndex(pointDimension - controlDimension),
									controlEndIndex(pointDimension),
									worldDimension(controlDimension)
								{
									assert(controlDimension<pointDimension);

									auto isControlIndex = [&](unsigned trajectoryIndex) {
										auto indexInPoint = (trajectoryIndex % pointDimension);
										return indexInPoint >= controlStartIndex && indexInPoint < controlEndIndex;
									};

									std::vector<double> trajectoryIndices(trajectoryDimension);
									std::iota(trajectoryIndices.begin(), trajectoryIndices.end(), 0);

									std::copy_if(trajectoryIndices.begin(), trajectoryIndices.end(), std::back_inserter(controlIndices), isControlIndex);
								};

			double operator()(const double* trajectoryPointer) const {
				double controlSquareSum = 0;

				const auto addToControlSquareSum = [&controlSquareSum, &trajectoryPointer] (const unsigned controlIndex)
										 { controlSquareSum += std::pow(trajectoryPointer[controlIndex], 2); };

				std::for_each(controlIndices.begin(), controlIndices.end(), addToControlSquareSum);

				return controlSquareSum;
			}  
	};

	// class GetToLastPointGoal{
	// 	const unsigned numberOfPoints;
	// 	const unsigned pointDimension;
	// 	const unsigned controlDimension;
	// 	const int trajectoryDimension;
	// 	const int positionDimension;
	// 	const int velocityDimension;
	// 	const int targetIndex;
	// 	const double targetValue;
	// 	const int goalTimeIndex;	// 1 - numberOfPoints

	// 	public:
	// 	GetToLastPointGoal(const unsigned numberOfPoints,
	// 						const unsigned pointDimension,
	// 						const unsigned controlDimension,
	// 						const int goalTimeIndex,
	// 						const double targetValue):
	// 						numberOfPoints(numberOfPoints),
	// 						pointDimension(pointDimension),
	// 						controlDimension(controlDimension),
	// 						trajectory(numberOfPoints * pointDimension),
	// 						positionDimension((pointDimension - controlDimension) / 2),
	// 						velocityDimension((pointDimension - controlDimension) / 2),
	// 						targetValue(targetValue),
	// 						goalTimeIndex(goalTimeIndex)
	// 						{
	// 							assert(goalTimeIndex <= numberOfPoints);
	// 							targetIndex = (goalTimeIndex - 1) * pointDimension + goalIndex;
	// 						}
	// 	double operator()(const double* trajectoryPointer) const 
	// 	{
	// 		double toGoalSquareSum = 0;

			
	// 	}
	// };
}




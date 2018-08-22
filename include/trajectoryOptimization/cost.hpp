#pragma once
#include <cmath>
#include <cassert>
#include <range/v3/view.hpp> 

#include "utilities.hpp"
#include "constraintNew.hpp"


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
		DynamicFunctionMujoco contactForce;
		public:
			GetControlSquareSum(const unsigned numberOfPoints,
								const unsigned pointDimension,
								const unsigned controlDimension,
								DynamicFunctionMujoco contactForce):
									numberOfPoints(numberOfPoints),
									pointDimension(pointDimension),
									controlDimension(controlDimension),
									trajectoryDimension(numberOfPoints * pointDimension),
									controlStartIndex(pointDimension - controlDimension),
									controlEndIndex(pointDimension),
									contactForce(contactForce),
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

				double contactPositionSquareSum = 0;
				const auto addToContactPositionSquareSum = [&] (const unsigned index)
				{
					const auto nowPosition = trajectoryPointer + index * pointDimension;
					const auto nowVelocity = nowPosition + worldDimension;
					const auto nowControl = nowVelocity + worldDimension;
					const auto force = contactForce(nowPosition, nowVelocity, nowControl);
					std::vector<mjtNum> contactPosition(force, force + worldDimension);
					for(int i = 0; i < contactPosition.size(); i ++)
						contactPositionSquareSum += std::pow(contactPosition[i], 2) * 10000;
				};
				std::vector<unsigned> numberOfPointsRange = ranges::view::ints((unsigned) 0, numberOfPoints);
				std::for_each(numberOfPointsRange.begin(), numberOfPointsRange.end(), addToContactPositionSquareSum);

				return controlSquareSum + contactPositionSquareSum;
			}  
	};
}//namespace




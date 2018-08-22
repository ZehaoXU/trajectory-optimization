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
									contactForce(contactForce)
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

				std::vector<constraint::ConstraintFunction> constraints;
				const unsigned kinematicViolationConstraintStartIndex = 0;
				const unsigned kinematicViolationConstraintEndIndex = kinematicViolationConstraintStartIndex + numberOfPoints - 1;
				const int numberVariablesX = pointDimension * numberOfPoints;

				constraints = constraint::applyKinematicViolationConstraintsUsingMujoco(constraints,
																				contactForce,
																				pointDimension,
																				pointDimension,
																				kinematicViolationConstraintStartIndex,
																				kinematicViolationConstraintEndIndex,
																				0.5);
				const constraint::ConstraintFunction stackedConstraintFunction = constraint::StackConstriants(numberVariablesX, constraints);
				std::vector<double> contactForceVector = stackedConstraintFunction(trajectoryPointer);

				double contactForceSum = 0;
				for(int i = 0; i < contactForceVector.size(); i++)
				{
					contactForceSum += contactForceVector[i];
				}

				return controlSquareSum + contactForceSum;
			}  
	};
}//namespace




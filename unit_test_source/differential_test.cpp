///
/// @file
/// @details A handful of test functions for testing the differential component of the drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "differential_test.h"
#include "test_kit.h"

#include "../source/racecar.h"
#include "../source/racecar_controller.h"
#include "../source/racecar_engine.h"
#include "../source/racecar_locked_differential.h"
#include "../source/racecar_wheel.h"

#include <array>

//--------------------------------------------------------------------------------------------------------------------//

struct LockedDifferentialTestBlob
{
	Racecar::Real gearRatio;
	Racecar::Real inputInertia;
	Racecar::Real inputAngularVelocity;
	Racecar::Real outputInertia;
	Racecar::Real outputAngularVelocity;
	Racecar::Real mExpectedInputAngularVelocity[2]; //After a single step, after a full second of steps.
	Racecar::Real mExpectedOutputAngularVelocity[2]; //After a single step, after a full second of steps.
};

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::LockedDifferentialTest(void)
{
	std::array<LockedDifferentialTestBlob, 2> tests {
		LockedDifferentialTestBlob{ 1.0,   10.0, 0.0,   10.0, 0.0,   { 0.1, 10.0 }, { 0.1, 10.0 } },
		LockedDifferentialTestBlob{ 4.0,   10.0, 0.0,   10.0, 0.0,   { 0.16, 16.0 }, { 0.04, 4.0 } },
	};

	for (const LockedDifferentialTestBlob& test : tests)
	{
		Racecar::ProgrammaticController racecarController;
		Racecar::ConstantEngine engine(test.inputInertia, 200.0, 0.0);
		Racecar::LockedDifferential lockedDifferential(test.outputInertia, test.gearRatio);

		engine.AddOutputSource(&lockedDifferential);
		lockedDifferential.SetInputSource(&engine);

		//Compute and test a single time-step of constant engine torque.
		racecarController.SetThrottlePosition(1.0f);
		engine.Simulate(racecarController, 0.01);
		lockedDifferential.Simulate(racecarController, 0.01);

		{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Single step.
			if (fabs(engine.GetAngularVelocity() - test.mExpectedInputAngularVelocity[0]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(lockedDifferential.GetAngularVelocity() - test.mExpectedOutputAngularVelocity[0]) > UnitTests::kTestElipson)
			{
				return false;
			}
		}

		//Simulate 1 seconds with the constant engine torque of 100Nm.
		racecarController.SetThrottlePosition(1.0f);
		for (int timer(10); timer < 1000; timer += 10)
		{
			engine.Simulate(racecarController, 0.01);
			lockedDifferential.Simulate(racecarController, 0.01);
		}

		{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Multiple-steps.
			if (fabs(engine.GetAngularVelocity() - test.mExpectedInputAngularVelocity[1]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(lockedDifferential.GetAngularVelocity() - test.mExpectedOutputAngularVelocity[1]) > UnitTests::kTestElipson)
			{
				return false;
			}
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

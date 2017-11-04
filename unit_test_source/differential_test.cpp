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

		//Simulate 1 seconds with the constant engine torque.
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

struct LockedDifferentialBrakingTestBlob
{
	Racecar::Real gearRatio;
	Racecar::Real engineInertia;
	Racecar::Real differentialInertia;
	Racecar::Real wheelInertia;
	Racecar::Real mExpectedEngineAngularVelocity[2]; //After a single step, after a full second of steps.
	Racecar::Real mExpectedDifferentialAngularVelocity[2]; //After a single step, after a full second of steps.
	Racecar::Real mExpectedWheelAngularVelocity[2]; //After a single step, after a full second of steps.
};

bool Racecar::UnitTests::LockedDifferentialBrakingTest(void)
{
	const Racecar::Real kTestTimeStep(0.01);

	std::array<LockedDifferentialBrakingTestBlob, 2> tests{
		LockedDifferentialBrakingTestBlob{ 1.0,   10.0, 10.0, 10.0, { 0.33333333, 33.33333333 }, { 0.33333333, 33.333333333 }, { 0.33333333, 33.33333333 } },
		LockedDifferentialBrakingTestBlob{ 4.0,   10.0, 10.0, 10.0, { 0.66666666, 66.66666666 }, { 0.16666666, 16.666666666 }, { 0.16666666, 16.66666666 } },
	};

	for (const LockedDifferentialBrakingTestBlob& test : tests)
	{
		Racecar::ProgrammaticController racecarController;
		Racecar::ConstantEngine engine(test.engineInertia, 1000.0, 0.0);
		Racecar::LockedDifferential lockedDifferential(test.differentialInertia, test.gearRatio);
		Racecar::Wheel wheel(test.wheelInertia / (0.25 * 0.25), 0.25);
		wheel.SetMaximumBrakingTorque(500.0);

		engine.AddOutputSource(&lockedDifferential);
		lockedDifferential.SetInputSource(&engine);
		lockedDifferential.AddOutputSource(&wheel);
		wheel.SetInputSource(&lockedDifferential);

		//Compute and test a single time-step of constant engine torque.
		racecarController.SetThrottlePosition(1.0f);
		engine.Simulate(racecarController, kTestTimeStep);
		lockedDifferential.Simulate(racecarController, kTestTimeStep);
		wheel.Simulate(racecarController, kTestTimeStep);

		{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Single step.
			if (fabs(engine.GetAngularVelocity() - test.mExpectedEngineAngularVelocity[0]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(lockedDifferential.GetAngularVelocity() - test.mExpectedDifferentialAngularVelocity[0]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(wheel.GetAngularVelocity() - test.mExpectedWheelAngularVelocity[0]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(engine.GetAngularVelocity() - lockedDifferential.GetAngularVelocity() * test.gearRatio) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(wheel.GetAngularVelocity() - lockedDifferential.GetAngularVelocity()) > UnitTests::kTestElipson)
			{
				return false;
			}
		}

		//Simulate 1 seconds with the constant engine torque of 100Nm.
		racecarController.SetThrottlePosition(1.0f);
		racecarController.SetBrakePosition(0.0f);
		for (int timer(10); timer < 1000; timer += 10)
		{
			engine.Simulate(racecarController, kTestTimeStep);
			lockedDifferential.Simulate(racecarController, kTestTimeStep);
			wheel.Simulate(racecarController, kTestTimeStep);
		}

		{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Multiple-steps.
			if (fabs(engine.GetAngularVelocity() - test.mExpectedEngineAngularVelocity[1]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(lockedDifferential.GetAngularVelocity() - test.mExpectedDifferentialAngularVelocity[1]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(wheel.GetAngularVelocity() - test.mExpectedWheelAngularVelocity[1]) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(engine.GetAngularVelocity() - lockedDifferential.GetAngularVelocity() * test.gearRatio) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(wheel.GetAngularVelocity() - lockedDifferential.GetAngularVelocity()) > UnitTests::kTestElipson)
			{
				return false;
			}
		}


		//Compute and test a single time-step of constant braking torque.
		racecarController.SetThrottlePosition(0.0f);
		racecarController.SetBrakePosition(1.0f);
		engine.Simulate(racecarController, kTestTimeStep);
		lockedDifferential.Simulate(racecarController, kTestTimeStep);
		wheel.Simulate(racecarController, kTestTimeStep);

		{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Single step.
			const Real expectedEngineAngularVelocity(test.mExpectedEngineAngularVelocity[1] - test.mExpectedEngineAngularVelocity[0] / 2.0);
			const Real expectedDifferentialAngularVelocity(test.mExpectedDifferentialAngularVelocity[1] - test.mExpectedDifferentialAngularVelocity[0] / 2.0);
			const Real expectedWheelAngularVelocity(test.mExpectedWheelAngularVelocity[1] - test.mExpectedWheelAngularVelocity[0] / 2.0);

			if (fabs(engine.GetAngularVelocity() - expectedEngineAngularVelocity) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(lockedDifferential.GetAngularVelocity() - expectedDifferentialAngularVelocity) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(wheel.GetAngularVelocity() - expectedWheelAngularVelocity) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(engine.GetAngularVelocity() - lockedDifferential.GetAngularVelocity() * test.gearRatio) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(wheel.GetAngularVelocity() - lockedDifferential.GetAngularVelocity()) > UnitTests::kTestElipson)
			{
				return false;
			}
		}


		//Simulate some time with the constant torque of braking from the wheel.
		racecarController.SetThrottlePosition(0.0f);
		racecarController.SetBrakePosition(1.0f);
		for (int timer(10); timer < 1000; timer += 10)
		{
			engine.Simulate(racecarController, kTestTimeStep);
			lockedDifferential.Simulate(racecarController, kTestTimeStep);
			wheel.Simulate(racecarController, kTestTimeStep);
		}

		{	//Make sure the engine is now spinning as fast as expected with given inertia / constant torque. Multiple-steps.
			if (fabs(engine.GetAngularVelocity() - test.mExpectedEngineAngularVelocity[1] / 2.0) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(lockedDifferential.GetAngularVelocity() - test.mExpectedDifferentialAngularVelocity[1] / 2.0) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(wheel.GetAngularVelocity() - test.mExpectedWheelAngularVelocity[1] / 2.0) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(engine.GetAngularVelocity() - lockedDifferential.GetAngularVelocity() * test.gearRatio) > UnitTests::kTestElipson)
			{
				return false;
			}
			if (fabs(wheel.GetAngularVelocity() - lockedDifferential.GetAngularVelocity()) > UnitTests::kTestElipson)
			{
				return false;
			}
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::LockedDifferentialUsageTest(void)
{
	//These tests not completely written, so the test themselves may be skewed/wrong.
	//{
	//	const Real initialInertia(10.0);
	//	LockedDifferential differential(initialInertia, 4.30);
	//	if (fabs(differential.GetInertia() - initialInertia) > UnitTests::kTestElipson)
	//	{
	//		return false;
	//	}
	//	if (fabs(differential.ComputeUpstreamInertia(differential) - initialInertia) > UnitTests::kTestElipson)
	//	{
	//		return false;
	//	}
	//	if (fabs(differential.ComputeDownstreamInertia(differential) - initialInertia) > UnitTests::kTestElipson)
	//	{
	//		return false;
	//	}
	//}

	//{
	//	LockedDifferential differential(1.0, 4.30);
	//	const Real initialInputInertia(10.0);
	//	const Real initialOutputInertia(10.0);
	//	RotatingBody inputBody(initialInputInertia);
	//	RotatingBody outputBody(initialOutputInertia);
	//	inputBody.AddOutputSource(&differential);
	//	differential.SetInputSource(&inputBody);
	//	differential.AddOutputSource(&outputBody);
	//	outputBody.SetInputSource(&differential);

	//	if (fabs(differential.GetInertia() - 1.0) > UnitTests::kTestElipson)
	//	{
	//		return false;
	//	}
	//	if (fabs(differential.ComputeUpstreamInertia(differential) - initialInputInertia) > UnitTests::kTestElipson)
	//	{
	//		return false;
	//	}
	//	if (fabs(differential.ComputeDownstreamInertia(differential) - initialOutputInertia) > UnitTests::kTestElipson)
	//	{
	//		return false;
	//	}

	//}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

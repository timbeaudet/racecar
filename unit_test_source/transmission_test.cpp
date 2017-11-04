///
/// @file
/// @details A handful of test functions for testing the transmission component of the drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "transmission_test.h"
#include "test_kit.h"

#include "../source/racecar.h"
#include "../source/racecar_controller.h"
#include "../source/racecar_engine.h"
#include "../source/racecar_locked_differential.h"
#include "../source/racecar_wheel.h"

#include <array>

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::TransmissionNeutralToFirstTest(void)
{
	const std::array<Racecar::Real, 6> forwardGearRatios{ 4.0, 3.0, 2.0, 1.0, 0.5, 0.0 };

	ProgrammaticController racecarController;
	ConstantEngine engine(10.0, 700.0, 0.0);
	Transmission gearbox(10.0, forwardGearRatios, -1.0);
	Wheel wheel(50.0 / (0.25 * 0.25), 0.25);

	engine.AddOutputSource(&gearbox);
	gearbox.SetInputSource(&engine);
	gearbox.AddOutputSource(&wheel);
	wheel.SetInputSource(&gearbox);

	if (Racecar::Gear::Neutral != gearbox.GetSelectedGear())
	{
		return false;
	}

	racecarController.SetThrottlePosition(1.0);
	for (int timer(0); timer < 1000; timer += 10)
	{
		engine.Simulate(racecarController, kTestFixedTimeStep);
		gearbox.Simulate(racecarController, kTestFixedTimeStep);
		wheel.Simulate(racecarController, kTestFixedTimeStep);
	}

	if (fabs(engine.GetAngularVelocity() - 70.0) > kTestElipson)
	{
		return false;
	}

	racecarController.SetThrottlePosition(0.0);
	racecarController.SetUpshift(true);
	//for (int timer(0); timer < 1000; timer += 10)
	{
		engine.Simulate(racecarController, kTestFixedTimeStep);
		gearbox.Simulate(racecarController, kTestFixedTimeStep);
		wheel.Simulate(racecarController, kTestFixedTimeStep);
	}

	if (fabs(engine.GetAngularVelocity() - gearbox.GetAngularVelocity() * forwardGearRatios[0]) > kTestElipson)
	{
		return false;
	}
	if (fabs(wheel.GetAngularVelocity() - gearbox.GetAngularVelocity()) > kTestElipson)
	{
		return false;
	}


	//gearbox.Simulate(racecarController, kTestFixedTimeStep);


	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::TransmissionBrakeInNeutralTest(void)
{
	const std::array<Racecar::Real, 6> forwardGearRatios{ 4.0, 3.0, 2.0, 1.0, 0.5, 0.0 };

	ProgrammaticController racecarController;
	ConstantEngine engine(10.0, 700.0, 0.0);
	Transmission gearbox(10.0, forwardGearRatios, -1.0);
	Wheel wheel(50.0 / (0.25 * 0.25), 0.25);
	wheel.SetMaximumBrakingTorque(600.0); //600Nm should take 1s to stop gearbox and wheel if their speed was 10rad/s

	engine.AddOutputSource(&gearbox);
	gearbox.SetInputSource(&engine);
	gearbox.AddOutputSource(&wheel);
	wheel.SetInputSource(&gearbox);

	if (Racecar::Gear::Neutral != gearbox.GetSelectedGear())
	{
		return false;
	}

	racecarController.SetThrottlePosition(1.0);
	racecarController.SetUpshift(true);
	for (int timer(0); timer < 1000; timer += 10)
	{
		engine.Simulate(racecarController, kTestFixedTimeStep);
		gearbox.Simulate(racecarController, kTestFixedTimeStep);
		wheel.Simulate(racecarController, kTestFixedTimeStep);

		racecarController.SetUpshift(false);
	}

	{
		if (fabs(engine.GetAngularVelocity() - gearbox.GetAngularVelocity() * forwardGearRatios[0]) > kTestElipson)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - gearbox.GetAngularVelocity()) > kTestElipson)
		{
			return false;
		}
	}
	
	//Hit the brake a little bit to start slowing down, while neutral.
	racecarController.SetThrottlePosition(0.0);
	racecarController.SetBrakePosition(1.0);
	racecarController.SetDownshift(true);
	for (int timer(0); timer < 50; timer += 10)
	{
		engine.Simulate(racecarController, kTestFixedTimeStep);
		gearbox.Simulate(racecarController, kTestFixedTimeStep);
		wheel.Simulate(racecarController, kTestFixedTimeStep);

		racecarController.SetDownshift(false);
	}

	{
		if (fabs(wheel.GetAngularVelocity() - gearbox.GetAngularVelocity()) > kTestElipson)
		{
			return false;
		}
	}

	//Do it again for a long time- should stop the wheel entirely, including gearbox output shaft.
	racecarController.SetThrottlePosition(0.0);
	racecarController.SetBrakePosition(1.0);
	racecarController.SetDownshift(true);
	for (int timer(0); timer < 5000; timer += 10)
	{
		engine.Simulate(racecarController, kTestFixedTimeStep);
		gearbox.Simulate(racecarController, kTestFixedTimeStep);
		wheel.Simulate(racecarController, kTestFixedTimeStep);

		racecarController.SetDownshift(false);
	}

	{
		if (fabs(wheel.GetAngularVelocity() - gearbox.GetAngularVelocity()) > kTestElipson)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

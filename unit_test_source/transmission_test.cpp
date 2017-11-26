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
#include "../source/racecar_clutch.h"
#include "../source/racecar_transmission.h"
#include "../source/racecar_wheel.h"

#include <fstream>
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
		engine.ControllerChange(racecarController);
		gearbox.ControllerChange(racecarController);
		wheel.ControllerChange(racecarController);

		engine.Simulate(kTestFixedTimeStep);
		gearbox.Simulate(kTestFixedTimeStep);
		wheel.Simulate(kTestFixedTimeStep);
	}

	if (fabs(engine.GetAngularVelocity() - 70.0) > kTestEpsilon)
	{
		return false;
	}

	racecarController.SetThrottlePosition(0.0);
	racecarController.SetUpshift(true);
	for (int timer(0); timer < 1000; timer += 10)
	{
		engine.ControllerChange(racecarController);
		gearbox.ControllerChange(racecarController);
		wheel.ControllerChange(racecarController);

		engine.Simulate(kTestFixedTimeStep);
		gearbox.Simulate(kTestFixedTimeStep);
		wheel.Simulate(kTestFixedTimeStep);
	}

	if (Racecar::Gear::First != gearbox.GetSelectedGear())
	{
		return false;
	}
	if (fabs(engine.GetAngularVelocity() - gearbox.GetAngularVelocity() * forwardGearRatios[0]) > kTestEpsilon)
	{
		return false;
	}
	if (fabs(wheel.GetAngularVelocity() - gearbox.GetAngularVelocity()) > kTestEpsilon)
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
		engine.ControllerChange(racecarController);
		gearbox.ControllerChange(racecarController);
		wheel.ControllerChange(racecarController);

		engine.Simulate(kTestFixedTimeStep);
		gearbox.Simulate(kTestFixedTimeStep);
		wheel.Simulate(kTestFixedTimeStep);

		racecarController.SetUpshift(false);
	}

	{
		if (fabs(engine.GetAngularVelocity() - gearbox.GetAngularVelocity() * forwardGearRatios[0]) > kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - gearbox.GetAngularVelocity()) > kTestEpsilon)
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
		engine.ControllerChange(racecarController);
		gearbox.ControllerChange(racecarController);
		wheel.ControllerChange(racecarController);

		engine.Simulate(kTestFixedTimeStep);
		gearbox.Simulate(kTestFixedTimeStep);
		wheel.Simulate(kTestFixedTimeStep);

		racecarController.SetDownshift(false);
	}

	{
		if (fabs(wheel.GetAngularVelocity() - gearbox.GetAngularVelocity()) > kTestEpsilon)
		{
			return false;
		}
	}

	//Do it again for a long time- should stop the wheel entirely, including gearbox output shaft.
	racecarController.SetThrottlePosition(0.0);
	racecarController.SetBrakePosition(1.0);
	for (int timer(0); timer < 5000; timer += 10)
	{
		engine.ControllerChange(racecarController);
		gearbox.ControllerChange(racecarController);
		wheel.ControllerChange(racecarController);

		engine.Simulate(kTestFixedTimeStep);
		gearbox.Simulate(kTestFixedTimeStep);
		wheel.Simulate(kTestFixedTimeStep);

	}

	{
		if (fabs(wheel.GetAngularVelocity() - gearbox.GetAngularVelocity()) > kTestEpsilon)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::TransmissionBrakeInReverseTest(void)
{
	const std::array<Racecar::Real, 6> forwardGearRatios{ 4.0, 3.0, 2.0, 1.0, 0.5, 0.0 };
	const Real reverseGearRatio(-2.5);

	ProgrammaticController racecarController;
	ConstantEngine engine(10.0, 700.0, 0.0);
	Clutch clutch(10.0, 100.0, 0.6, 0.4);
	Transmission gearbox(10.0, forwardGearRatios, reverseGearRatio);
	Wheel wheel(50.0 / (0.25 * 0.25), 0.25);
	wheel.SetMaximumBrakingTorque(6000.0); //600Nm should take 1s to stop gearbox and wheel if their speed was 10rad/s

	engine.AddOutputSource(&clutch);
	clutch.SetInputSource(&engine);
	clutch.AddOutputSource(&gearbox);
	gearbox.SetInputSource(&clutch);
	gearbox.AddOutputSource(&wheel);
	wheel.SetInputSource(&gearbox);

	engine.SetAngularVelocity(100.0);

	racecarController.SetThrottlePosition(0.0);
	racecarController.SetBrakePosition(0.0);
	racecarController.SetDownshift(true);

	auto stepFunction = [&]() {
		engine.ControllerChange(racecarController);
		clutch.ControllerChange(racecarController);
		gearbox.ControllerChange(racecarController);
		wheel.ControllerChange(racecarController);

		engine.Simulate(kTestFixedTimeStep);
		clutch.Simulate(kTestFixedTimeStep);
		gearbox.Simulate(kTestFixedTimeStep);
		wheel.Simulate(kTestFixedTimeStep);
	};

	stepFunction();
	racecarController.SetUpshift(false);

	const Real reverseWheelSpeed(fabs(wheel.GetAngularVelocity()));

	{
		if (Racecar::Gear::Reverse != gearbox.GetSelectedGear())
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity() - gearbox.GetAngularVelocity() * reverseGearRatio) > kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - gearbox.GetAngularVelocity()) > kTestEpsilon)
		{
			return false;
		}
	}
	
	racecarController.SetBrakePosition(1.0f);
	for (int timer(0); timer < 200; timer += 10)
	{
		const Real previousWheelVelocity(wheel.GetAngularVelocity());
		stepFunction();
		if (fabs(previousWheelVelocity) < fabs(wheel.GetAngularVelocity()))
		{	//The wheel is sped up, NOT slowing down.
			return false;
		}
	}

	{
		if (Racecar::Gear::Reverse != gearbox.GetSelectedGear())
		{
			return false;
		}
		if (fabs(clutch.GetAngularVelocity() - gearbox.GetAngularVelocity() * reverseGearRatio) > kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - gearbox.GetAngularVelocity()) > kTestEpsilon)
		{
			return false;
		}
		if (reverseWheelSpeed < fabs(wheel.GetAngularVelocity()))
		{	//The wheel is speeding up, NOT slowing down.
			return false;
		}
	}

	//This should be enough time to stop the wheel and transmission, (and engine since we are in gear!)
	for (int timer(0); timer < 20000; timer += 10)
	{
		const Real previousWheelVelocity(wheel.GetAngularVelocity());
		stepFunction();
		if (fabs(previousWheelVelocity) < fabs(wheel.GetAngularVelocity()))
		{	//The wheel is sped up, NOT slowing down.
			return false;
		}
	}

	{
		if (Racecar::Gear::Reverse != gearbox.GetSelectedGear())
		{
			return false;
		}
		if (fabs(engine.GetAngularVelocity() - gearbox.GetAngularVelocity() * reverseGearRatio) > kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - gearbox.GetAngularVelocity()) > kTestEpsilon)
		{
			return false;
		}
		if (reverseWheelSpeed < fabs(wheel.GetAngularVelocity()))
		{	//The wheel is speeding up, NOT slowing down.
			return false;
		}
		if (fabs(wheel.GetAngularVelocity()) > kTestEpsilon)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

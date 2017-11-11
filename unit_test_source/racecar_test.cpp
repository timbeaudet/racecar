///
/// @file
/// @details A handful of test functions for testing the entire drive-train of the racecar as a full unit.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_test.h"
#include "test_kit.h"

#include "../source/racecar.h"
#include "../source/racecar_controller.h"
#include "../source/racecar_engine.h"
#include "../source/racecar_clutch.h"
#include "../source/racecar_transmission.h"
#include "../source/racecar_locked_differential.h"
#include "../source/racecar_wheel.h"
#include "../source/racecar_body.h"

#include <array>
#include <fstream>

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::RacecarAccelerationTest(void)
{
	const float radius(1.0);
	const Real finalDriveRatio(1.0);
	const std::array<Real, 6> forwardGearRatios{ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
	const Real reverseGearRatio(-1.0);

	Racecar::ProgrammaticController racecarController;

	TorqueCurve engineTorqueCurve;
	engineTorqueCurve.AddPlotPoint(0.0, 1000.0);
	engineTorqueCurve.AddPlotPoint(10000, 1000.0);
	engineTorqueCurve.NormalizeTorqueCurve();

	Engine engine(10.0, engineTorqueCurve);
	Clutch clutch(10.0, 10000.0);
	Transmission gearbox(10.0, forwardGearRatios, reverseGearRatio);
	LockedDifferential differential(10.0, finalDriveRatio);	
	Wheel wheel(10.0 / (radius * radius), radius);

	//Rotating Moment of Inertia: 50kg*m^2
	RacecarBody racecarBody(50);;// / (wheel.GetRadius() * wheel.GetRadius()));

	//Link up all the components:
	engine.AddOutputSource(&clutch);
	clutch.SetInputSource(&engine);
	clutch.AddOutputSource(&gearbox);
	gearbox.SetInputSource(&clutch);
	gearbox.AddOutputSource(&differential);
	differential.SetInputSource(&gearbox);
	differential.AddOutputSource(&wheel);
	wheel.SetInputSource(&differential);
	wheel.SetRacecarBody(&racecarBody);
	racecarBody.SetWheel(0, &wheel);

	//Ensure the car is on the ground.
	wheel.SetOnGround(true, -1.0);

	racecarController.SetThrottlePosition(1.0);
	racecarController.SetBrakePosition(0.0);
	racecarController.SetClutchPosition(0.0);
	racecarController.SetShifterPosition(Racecar::Gear::First);

	engine.SetAngularVelocity(0.0);

	std::ofstream outFile("data/outputs/accel2.txt");

	engine.ControllerChange(racecarController);
	clutch.ControllerChange(racecarController);
	gearbox.ControllerChange(racecarController);
	differential.ControllerChange(racecarController);
	wheel.ControllerChange(racecarController);
	racecarBody.ControllerChange(racecarController);

	engine.Simulate(kTestFixedTimeStep);
	clutch.Simulate(kTestFixedTimeStep);
	gearbox.Simulate(kTestFixedTimeStep);
	differential.Simulate(kTestFixedTimeStep);
	wheel.Simulate(kTestFixedTimeStep);
	racecarBody.Simulate(kTestFixedTimeStep);


	{
		if (fabs(wheel.GetAngularVelocity() - 10.0) > kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetLinearVelocity() - 17.24137931) > kTestEpsilon)
		{
			return false;
		}
	}


	for (int timer(0); timer < 1000; timer += 10)
	{
		engine.ControllerChange(racecarController);
		clutch.ControllerChange(racecarController);
		gearbox.ControllerChange(racecarController);
		differential.ControllerChange(racecarController);
		wheel.ControllerChange(racecarController);
		racecarBody.ControllerChange(racecarController);

		engine.Simulate(kTestFixedTimeStep);
		clutch.Simulate(kTestFixedTimeStep);
		gearbox.Simulate(kTestFixedTimeStep);
		differential.Simulate(kTestFixedTimeStep);
		wheel.Simulate(kTestFixedTimeStep);
		racecarBody.Simulate(kTestFixedTimeStep);

		outFile << timer + 10 << "\t" << engine.GetAngularVelocity() << "\t" << wheel.GetAngularVelocity() << "\t" << wheel.GetWheelSpeedMPH()
			<< "\t" << wheel.GetLinearVelocity() << std::endl;
		outFile.flush();
	}

	{
		if (fabs(wheel.GetAngularVelocity() - 10.0) > kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetLinearVelocity() - 17.24137931) > kTestEpsilon)
		{
			return false;
		}
	}

	outFile.close();

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::RacecarZeroToSixtyTest(void)
{
	return true;

	const Real finalDriveRatio(4.3);
	const std::array<Real, 6> forwardGearRatios{ 3.163, 1.0, 1.0, 1.0, 1.0, 1.0 };
	const Real reverseGearRatio(-1.0);

	Racecar::ProgrammaticController racecarController;

	//	ConstantEngine engine(10.0, 1000.0, 0.0);

	TorqueCurve engineTorqueCurve;
	engineTorqueCurve.AddPlotPoint(0.0, 72.319547676992);
	engineTorqueCurve.AddPlotPoint(25000, 72.319547676992);
	engineTorqueCurve.NormalizeTorqueCurve();

	Engine engine(5.0, engineTorqueCurve);
	Clutch clutch(0.96, 10000.0);
	Transmission gearbox(0.24, forwardGearRatios, reverseGearRatio);
	LockedDifferential differential(0.3, finalDriveRatio);

	const float radius(0.2794); //11 inches in meters.
	Wheel wheel(0.5 / (radius * radius), radius);

	//Rotating Moment of Inertia: 7 kg*m^2
	RacecarBody racecarBody(1042); //Mass of miata without a driver! Who's driving?

	//Link up all the components:
	engine.AddOutputSource(&clutch);
	clutch.SetInputSource(&engine);
	clutch.AddOutputSource(&gearbox);
	gearbox.SetInputSource(&clutch);
	gearbox.AddOutputSource(&differential);
	differential.SetInputSource(&gearbox);
	differential.AddOutputSource(&wheel);
	wheel.SetInputSource(&differential);
	wheel.SetRacecarBody(&racecarBody);
	racecarBody.SetWheel(0, &wheel);

	//Ensure the car is on the ground.
	wheel.SetOnGround(true, Racecar::Wheel::kInfiniteFriction);

	racecarController.SetThrottlePosition(1.0);
	racecarController.SetBrakePosition(0.0);
	racecarController.SetClutchPosition(0.0);
	racecarController.SetShifterPosition(Racecar::Gear::First);

	engine.SetAngularVelocity(0.0);

	std::ofstream outFile("data/outputs/accel.txt");

	engine.ControllerChange(racecarController);
	clutch.ControllerChange(racecarController);
	gearbox.ControllerChange(racecarController);
	differential.ControllerChange(racecarController);
	wheel.ControllerChange(racecarController);
	racecarBody.ControllerChange(racecarController);

	Racecar::Real rotatingInertia = wheel.ComputeUpstreamInertia();
	if (fabs(rotatingInertia - 82.893364) > kTestEpsilon)
	{
		return false;
	}

	engine.Simulate(kTestFixedTimeStep);
	clutch.Simulate(kTestFixedTimeStep);
	gearbox.Simulate(kTestFixedTimeStep);
	differential.Simulate(kTestFixedTimeStep);
	wheel.Simulate(kTestFixedTimeStep);
	racecarBody.Simulate(kTestFixedTimeStep);

	{
		if (fabs(wheel.GetLinearVelocity() - 0.033528) > kTestEpsilon)
		{
			return false;
		}
		if (fabs(racecarBody.GetLinearVelocity() - 0.033528) > kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - 0.05988993753) > kTestEpsilon)
		{
			return false;
		}
	}





	for (int timer(10); timer < 8000; timer += 10)
	{
		engine.ControllerChange(racecarController);
		clutch.ControllerChange(racecarController);
		gearbox.ControllerChange(racecarController);
		differential.ControllerChange(racecarController);
		wheel.ControllerChange(racecarController);
		racecarBody.ControllerChange(racecarController);

		Racecar::Real rotatingInertia = wheel.ComputeUpstreamInertia();
		if (fabs(rotatingInertia - 82.893364) > kTestEpsilon)
		{
			return false;
		}

		engine.Simulate(kTestFixedTimeStep);
		clutch.Simulate(kTestFixedTimeStep);
		gearbox.Simulate(kTestFixedTimeStep);
		differential.Simulate(kTestFixedTimeStep);
		wheel.Simulate(kTestFixedTimeStep);
		racecarBody.Simulate(kTestFixedTimeStep);

		outFile << timer + 10 << "\t" << engine.GetAngularVelocity() << "\t" << wheel.GetAngularVelocity() << "\t" << wheel.GetWheelSpeedMPH()
			<< "\t" << wheel.GetLinearVelocity() << std::endl;
		outFile.flush();
	}

	{
		if (fabs(wheel.GetLinearVelocity() - 26.8224) > kTestEpsilon)
		{
			return false;
		}
		if (fabs(racecarBody.GetLinearVelocity() - 26.8224) > kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - 47.91195002) > kTestEpsilon)
		{
			return false;
		}
	}

	outFile.close();

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

///
/// @file
/// @details A handful of test functions for testing the wheel component of the drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "wheel_test.h"
#include "test_kit.h"

#include "../source/racecar.h"
#include "../source/racecar_controller.h"
#include "../source/racecar_wheel.h"
#include "../source/racecar_engine.h"
#include "../source/racecar_clutch.h"

#include <array>

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::WheelBrakingTest(void)
{
	Racecar::ProgrammaticController racecarController;
	Racecar::Wheel wheel(8.0, 0.25); //0.5kg*m^2

	wheel.SetAngularVelocity(40.0); //rad/s  (must be positive for < 0.0 checks to work.
	wheel.SetMaximumBrakingTorque(20.0); //Nm

	//5Nm, should lose 0.1rad/s per time step.
	racecarController.SetBrakePosition(0.25);
	wheel.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	{	//Wheel speed should now be 40.0 - 0.1 rad/sec
		if (fabs(wheel.GetAngularVelocity() - 39.9) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (wheel.GetAngularVelocity() < 0.0)
		{
			return false;
		}
	}

	//10Nm, should lose 0.2rad/s per time step.
	racecarController.SetBrakePosition(0.5);
	wheel.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	{	//Wheel speed should now be 40.0 - 0.1 - 0.2 rad/sec
		if (fabs(wheel.GetAngularVelocity() - 39.7) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (wheel.GetAngularVelocity() < 0.0)
		{
			return false;
		}
	}

	//20Nm, should lose 0.4rad/s per time step.
	racecarController.SetBrakePosition(1.0);
	wheel.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	{	//Wheel speed should now be 40.0 - 0.1 - 0.2 - 0.4 rad/sec
		if (fabs(wheel.GetAngularVelocity() - 39.3) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (wheel.GetAngularVelocity() < 0.0)
		{
			return false;
		}
	}

	//After 0.99 seconds the wheel should be going .4 + .3 + .2 rad/s due to above tests.
	for (int timer(30); timer < 1010; timer += 10)
	{
		wheel.ControllerChange(racecarController);
		wheel.Simulate(kTestFixedTimeStep);
	}
	{	//Wheel speed should now be  .1 rad/s due to above tests and 1.01 second simulation time.
		if (fabs(wheel.GetAngularVelocity() - 0.1) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (wheel.GetAngularVelocity() < 0.0)
		{
			return false;
		}
	}

	wheel.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	wheel.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	{	//Wheel speed should stay at 0.
		if (fabs(wheel.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (wheel.GetAngularVelocity() < 0.0)
		{
			return false;
		}
	}

	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ControllerChange(racecarController);
		wheel.Simulate(kTestFixedTimeStep);
	}
	{	//Wheel speed should stay at 0.
		if (fabs(wheel.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (wheel.GetAngularVelocity() < 0.0)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::WheelNegativeBrakingTest(void)
{
	Racecar::ProgrammaticController racecarController;
	Racecar::Wheel wheel(8.0, 0.25); //0.5kg*m^2

	wheel.SetAngularVelocity(-40.0); //rad/s  (must be positive for < 0.0 checks to work.
	wheel.SetMaximumBrakingTorque(20.0); //Nm

	//5Nm, should lose 0.1rad/s per time step.
	racecarController.SetBrakePosition(0.25);
	wheel.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	{	//Wheel speed should now be 40.0 - 0.1 rad/sec
		if (fabs(wheel.GetAngularVelocity() + 39.9) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (wheel.GetAngularVelocity() > 0.0)
		{
			return false;
		}
	}

	//10Nm, should lose 0.2rad/s per time step.
	racecarController.SetBrakePosition(0.5);
	wheel.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	{	//Wheel speed should now be 40.0 - 0.1 - 0.2 rad/sec
		if (fabs(wheel.GetAngularVelocity() + 39.7) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (wheel.GetAngularVelocity() > 0.0)
		{
			return false;
		}
	}

	//20Nm, should lose 0.4rad/s per time step.
	racecarController.SetBrakePosition(1.0);
	wheel.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	{	//Wheel speed should now be 40.0 - 0.1 - 0.2 - 0.4 rad/sec
		if (fabs(wheel.GetAngularVelocity() + 39.3) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (wheel.GetAngularVelocity() > 0.0)
		{
			return false;
		}
	}

	//After 0.99 seconds the wheel should be going .4 + .3 + .2 rad/s due to above tests.
	for (int timer(30); timer < 1010; timer += 10)
	{
		wheel.ControllerChange(racecarController);
		wheel.Simulate(kTestFixedTimeStep);
	}
	{	//Wheel speed should now be  .1 rad/s due to above tests and 1.01 second simulation time.
		if (fabs(wheel.GetAngularVelocity() + 0.1) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (wheel.GetAngularVelocity() > 0.0)
		{
			return false;
		}
	}

	wheel.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	wheel.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	{	//Wheel speed should stay at 0.
		if (fabs(wheel.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ControllerChange(racecarController);
		wheel.Simulate(kTestFixedTimeStep);
	}
	{	//Wheel speed should stay at 0.
		if (fabs(wheel.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::WheelAndAxleBrakingTest(void)
{
	Racecar::ProgrammaticController racecarController;
	Racecar::Wheel wheel(8.0, 0.25); //0.5kg*m^2
	Racecar::RotatingBody axle(4.5); //kg*m^2

	wheel.SetInputSource(&axle);
	axle.AddOutputSource(&wheel);

	axle.SetAngularVelocity(40.0);
	wheel.SetAngularVelocity(40.0); //rad/s
	wheel.SetMaximumBrakingTorque(200.0); //Nm

	//5Nm, should lose 0.1rad/s per time step.
	racecarController.SetBrakePosition(0.25);
	wheel.ControllerChange(racecarController);
	axle.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	axle.Simulate(kTestFixedTimeStep);
	{	//Wheel speed should now be 40.0 - 0.1 rad/sec
		if (fabs(wheel.GetAngularVelocity() - 39.9) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - axle.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	//10Nm, should lose 0.2rad/s per time step.
	racecarController.SetBrakePosition(0.5);

	wheel.ControllerChange(racecarController);
	axle.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	axle.Simulate(kTestFixedTimeStep);
	{	//Wheel speed should now be 40.0 - 0.1 - 0.2 rad/sec
		if (fabs(wheel.GetAngularVelocity() - 39.7) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - axle.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	//20Nm, should lose 0.4rad/s per time step.
	racecarController.SetBrakePosition(1.0);

	wheel.ControllerChange(racecarController);
	axle.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	axle.Simulate(kTestFixedTimeStep);
	{	//Wheel speed should now be 40.0 - 0.1 - 0.2 - 0.4 rad/sec
		if (fabs(wheel.GetAngularVelocity() - 39.3) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - axle.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	//After 0.99 seconds the wheel should be going .4 + .3 + .2 rad/s due to above tests.
	for (int timer(30); timer < 1010; timer += 10)
	{
		wheel.ControllerChange(racecarController);
		axle.ControllerChange(racecarController);

		wheel.Simulate(kTestFixedTimeStep);
		axle.Simulate(kTestFixedTimeStep);
	}
	{	//Wheel speed should now be  .1 rad/s due to above tests and 1.01 second simulation time.
		if (fabs(wheel.GetAngularVelocity() - 0.1) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - axle.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}


	wheel.ControllerChange(racecarController);
	axle.ControllerChange(racecarController);
	wheel.Simulate(kTestFixedTimeStep);
	axle.Simulate(kTestFixedTimeStep);

	wheel.Simulate(kTestFixedTimeStep);
	axle.Simulate(kTestFixedTimeStep);
	wheel.ControllerChange(racecarController);
	axle.ControllerChange(racecarController);

	{	//Wheel speed should stay at 0.
		if (fabs(wheel.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - axle.GetAngularVelocity()) > UnitTests::kTestEpsilon)
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::WheelClutchAndEngineBrakingTest(void)
{
	Racecar::ProgrammaticController racecarController;
	Racecar::Wheel wheel(8.0, 0.25); //0.5kg*m^2
	Racecar::Clutch clutch(2.5, 100, 0.6, 0.4); //kg*m^2
	Racecar::Engine engine(2.0, Racecar::TorqueCurve::MiataTorqueCurve()); //kg*m^2

	engine.AddOutputSource(&clutch);
	clutch.SetInputSource(&engine);
	clutch.AddOutputSource(&wheel);
	wheel.SetInputSource(&clutch);

	engine.SetAngularVelocity(RevolutionsMinuteToRadiansSecond(1000));
	clutch.SetAngularVelocity(0.0);
	wheel.SetAngularVelocity(0.0); //rad/s
	wheel.SetMaximumBrakingTorque(200.0); //Nm

	racecarController.SetBrakePosition(1.0f);
	racecarController.SetClutchPosition(0.5f);
	racecarController.SetThrottlePosition(0.10f);

	for (int timer(0); timer < 200000; timer += 10)
	{
		engine.ControllerChange(racecarController);
		clutch.ControllerChange(racecarController);
		wheel.ControllerChange(racecarController);

		engine.Simulate(kTestFixedTimeStep);
		clutch.Simulate(kTestFixedTimeStep);
		wheel.Simulate(kTestFixedTimeStep);

		if (wheel.GetAngularVelocity() < -kTestEpsilon || engine.GetAngularVelocity() < -kTestEpsilon || clutch.GetAngularVelocity() < -kTestEpsilon ||
			wheel.GetAngularVelocity() > 10000.0 || engine.GetAngularVelocity() > 10000.0 || clutch.GetAngularVelocity() > 10000.0)
		{	//This was causing a 0.0 / 0.01 issue that resulted in NaN to be generated.
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

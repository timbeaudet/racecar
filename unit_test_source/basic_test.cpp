///
/// @file
/// @details Defines the functions that perform the basic tests for the racecar drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "basic_test.h"
#include "test_kit.h"
#include "engine_test.h"
#include "clutch_test.h"
#include "wheel_test.h"
#include "differential_test.h"
#include "transmission_test.h"
#include "linear_motion_test.h"
#include "racecar_test.h"

#include "../source/racecar.h"
#include "../source/racecar_controller.h"
#include "../source/racecar_wheel.h"
#include "../source/racecar_locked_differential.h"

#include <cstdio>

//--------------------------------------------------------------------------------------------------------------------//

using Racecar::Real;
using Racecar::UnitTests::kTestEpsilon;
using Racecar::UnitTests::kTestFixedTimeStep;

bool ConstructionTest(void);
bool ConstantTorqueTest(void);
bool GearReductionTest(void);

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::PerformBasicTests(void)
{
	log_test("\n\n...................\nWelcome to scrutineering\nIt is time to check over the components of your racecar...\nGood Luck!\n...................\n\n");

	PerformTest(ConstructionTest, "Constructing a Rotating Body");
	PerformTest(ConstantTorqueTest, "Applying Constant Torque");
	//PerformTest(GearReductionTest, "Constant Torque through Gear Reduction");
	PerformTest(WheelWithLinearMotion, "Wheel with Linear Motion");
	PerformTest(RacecarWithLinearMotion, "Racecar with Linear Motion");
	PerformTest(EngineWheelCarLinearMotion, "Engine Wheel Racecar Linear Motion");
	PerformTest(EngineGearboxWheelCarLinearMotion, "Engine Gearbox Wheel Racecar Linear Motion");

	PerformTest(SpinningWheelsReleasedFromJack, "Spinning Wheels Released From Jack");
	PerformTest(FlyingCarHitsTrack, "Flying Car Hits Track");

	PerformTest(BasicEngineTest, "Basic Engine Test");
	PerformTest(EngineTorqueTest, "Engine Torque Test");
	PerformTest(WheelBrakingTest, "Wheel Braking Test");
	PerformTest(WheelNegativeBrakingTest, "Wheel Negative Braking Test");
	PerformTest(WheelAndAxleBrakingTest, "Wheel And Axle Braking Test");
	PerformTest(WheelClutchAndEngineBrakingTest, "Wheel Clutch And Engine Braking Test"); //Looking for potential NaN
	PerformTest(EngineClutchWheelThrottleTest, "Engine, Clutch Wheel Throttle Test");     //Checking to ensure the clutch/wheel don't spin faster than engine.
	PerformTest(EngineClutchWheelBrakingTest, "Engine, Clutch Wheel Braking Test");       //Checking if brakes slow engine with clutch disengaged.
	PerformTest(EngineClutchWheelMismatchTest, "Engine, Clutch Wheel Mismatch Test");     //Ensures the wheel and clutch remain same speeds while trying to match engine speed.
	PerformTest(EngineWithConnectionTest, "Engine With Connection Test");
	PerformTest(ClutchInputTest, "Clutch Input Test");
	PerformTest(SlippingClutchTest, "Slipping Clutch Test");
	PerformTest(LockedDifferentialTest, "Locked Differential Test");
	PerformTest(LockedDifferentialBrakingTest, "Locked Differential Braking Test");
	PerformTest(LockedDifferentialUsageTest, "Locked Differential Usage Test");
	PerformTest(TransmissionNeutralToFirstTest, "Transmission Neutral to First Test");
	PerformTest(TransmissionBrakeInNeutralTest, "Transmission Brake in Neutral Test");
	PerformTest(TransmissionBrakeInReverseTest, "Transmission Brake in Reverse Test");

	//PerformTest(RacecarAccelerationTest, "Racecar Acceleration Test");
	//PerformTest(RacecarZeroToSixtyTest, "Racecar Zero To Sixty Test");

	if (true == Racecar::UnitTests::sAllTestsPassed)
	{
		log_test("Your racecar has successfully passed technical inspection.\nYou may now go racing!\n\n");
	}
	else
	{
		log_test("Your racecar has not passed technical inspection.\nYou must fix it proper before racing!\n\n");
	}

	return (true == sAllTestsPassed);
}

//--------------------------------------------------------------------------------------------------------------------//

bool ConstructionTest(void)
{
	Racecar::RotatingBody rotatingMass(10.0); //10kg-m^2.
	Racecar::UnitTests::ExpectedValue(rotatingMass.GetInertia(), 10.0, "Mass1 accessing expected moments of inertia.");

	Racecar::RotatingBody rotatingMass2(Racecar::ComputeInertiaMetric(10.0, 1.0));
	Racecar::UnitTests::ExpectedValue(rotatingMass.GetInertia(), rotatingMass2.GetInertia(), "Mass2 accessing expected moments of inertia.");

	const Real startingVelocity(rotatingMass.GetAngularVelocity());
	Racecar::UnitTests::ExpectedValue(startingVelocity, 0.0, "Rotating body has non-Zero angular velocity after construction. Was: %f\n", startingVelocity);

	Racecar::UnitTests::ExpectedValue(42, 0, "This is expected to fail!");

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool ConstantTorqueTest(void)
{
	Racecar::DoNothingController racecarController;
	Racecar::Wheel wheel(8.0, 0.25); //0.5kg-m^2.

	//Simulate 1 second of applying a 200Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyDownstreamAngularImpulse(200.0 * 0.01); //200nm torque
		wheel.ControllerChange(racecarController);
		wheel.Simulate(kTestFixedTimeStep);
	}

	Racecar::UnitTests::ExpectedValue(wheel.GetAngularVelocity(), 400.0, "1s constant positive downstream torque.");
	

	//Simulate 1 second of applying a negative 200Nm torque to the rotating body, to stop the wheel.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyDownstreamAngularImpulse(-200.0 * 0.01);
		wheel.ControllerChange(racecarController);
		wheel.Simulate(kTestFixedTimeStep);
	}

	Racecar::UnitTests::ExpectedValue(wheel.GetAngularVelocity(), 0.0, "1s constant negative downstream torque.");


	//Simulate 1 second of applying a 200Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyUpstreamAngularImpulse(200.0 * 0.01);
		wheel.ControllerChange(racecarController);
		wheel.Simulate(kTestFixedTimeStep);
	}

	Racecar::UnitTests::ExpectedValue(wheel.GetAngularVelocity(), 400.0, "1s constant positive upstream torque.");


	//Simulate 1 second of applying a negative 10Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyUpstreamAngularImpulse(-200.0 * 0.01);
		wheel.ControllerChange(racecarController);
		wheel.Simulate(kTestFixedTimeStep);
	}

	Racecar::UnitTests::ExpectedValue(wheel.GetAngularVelocity(), 0.0, "1s constant negative upstream torque.");

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool GearReductionTest(void)
{
	//const Real kGearReduction(4.0);

	//Racecar::DoNothingController racecarController;
	//Racecar::RotatingBody engineMass(10.0); //10kg-m^2.
	//Racecar::RotatingBody wheelMass(10.0);
	//Racecar::LockedDifferential lockedDifferential(10.0, kGearReduction); //4:1 ratio with inertia of 10kg-m^2

	//engineMass.AddOutputSource(&lockedDifferential);
	//lockedDifferential.SetInputSource(&engineMass);
	//lockedDifferential.AddOutputSource(&wheelMass);
	//wheelMass.SetInputSource(&lockedDifferential);

	////Simulate 1 second of applying a 30Nm torque to the rotating body.
	//for (int timer(0); timer < 1000; timer += 10)
	//{
	//	engineMass.ControllerChange(racecarController);
	//	lockedDifferential.ControllerChange(racecarController);
	//	wheelMass.ControllerChange(racecarController);

	//	engineMass.ApplyDownstreamAngularImpulse((-10 + (20 / 4.0)) * 0.01);
	//	engineMass.Simulate(kTestFixedTimeStep);
	//	lockedDifferential.Simulate(kTestFixedTimeStep);
	//	wheelMass.Simulate(kTestFixedTimeStep);
	//}

	//const Real expectedDownEngineVelocity(1.0);
	//const Real finalDownEngineVelocity(engineMass.GetAngularVelocity());
	//const Real expectedDownWheelVelocity(expectedDownEngineVelocity / kGearReduction);
	//const Real finalDownWheelVelocity(wheelMass.GetAngularVelocity());
	//if (fabs(finalDownWheelVelocity - expectedDownWheelVelocity) > kTestEpsilon ||
	//	fabs(finalDownEngineVelocity - expectedDownEngineVelocity) > kTestEpsilon)
	//{
	//	log_test("Failed: 1 second constant torque, expected 1rad/sec finalVelocity.\n");
	//	return false;
	//}

	////Simulate 1 second of applying a 30Nm torque to the rotating body.
	//for (int timer(0); timer < 1000; timer += 10)
	//{
	//	engineMass.ControllerChange(racecarController);
	//	lockedDifferential.ControllerChange(racecarController);
	//	wheelMass.ControllerChange(racecarController);

	//	engineMass.ApplyDownstreamAngularImpulse((-10 + (20 / 4.0)) * 0.01);
	//	engineMass.Simulate(kTestFixedTimeStep); //Simulates 10ms of action.
	//	lockedDifferential.Simulate(kTestFixedTimeStep);
	//	wheelMass.Simulate(kTestFixedTimeStep);
	//}

	//const Racecar::Real negativeDownEngineVelocity(engineMass.GetAngularVelocity());
	//const Racecar::Real negativeDownWheelVelocity(wheelMass.GetAngularVelocity());
	//if (fabs(negativeDownEngineVelocity) > kTestEpsilon || fabs(negativeDownWheelVelocity) > kTestEpsilon)
	//{
	//	log_test("Failed: 1 second constant torque, expected 1rad/sec finalVelocity.\n");
	//	return false;
	//}

	/////////////

	////Simulate 1 second of applying a 30Nm torque to the rotating body.
	//for (int timer(0); timer < 1000; timer += 10)
	//{
	//	engineMass.ControllerChange(racecarController);
	//	lockedDifferential.ControllerChange(racecarController);
	//	wheelMass.ControllerChange(racecarController);

	//	wheelMass.ApplyUpstreamAngularImpulse(30.0 * 0.01);
	//	engineMass.Simulate(kTestFixedTimeStep);
	//	lockedDifferential.Simulate(kTestFixedTimeStep);
	//	wheelMass.Simulate(kTestFixedTimeStep);
	//}

	//const Real expectedUpEngineVelocity(1.0);
	//const Real finalUpEngineVelocity(engineMass.GetAngularVelocity());
	//const Real expectedUpWheelVelocity(expectedUpEngineVelocity / kGearReduction);
	//const Real finalUpWheelVelocity(wheelMass.GetAngularVelocity());
	//if (fabs(finalUpWheelVelocity - expectedUpWheelVelocity) > kTestEpsilon ||
	//	fabs(finalUpEngineVelocity - expectedUpEngineVelocity) > kTestEpsilon)
	//{
	//	log_test("Failed: 1 second constant torque, expected 1rad/sec finalVelocity.\n");
	//	return false;
	//}

	////Simulate 1 second of applying a 30Nm torque to the rotating body.
	//for (int timer(0); timer < 1000; timer += 10)
	//{
	//	engineMass.ControllerChange(racecarController);
	//	lockedDifferential.ControllerChange(racecarController);
	//	wheelMass.ControllerChange(racecarController);

	//	wheelMass.ApplyUpstreamAngularImpulse(-30.0 * 0.01);
	//	engineMass.Simulate(kTestFixedTimeStep); //Simulates 10ms of action.
	//	lockedDifferential.Simulate(kTestFixedTimeStep);
	//	wheelMass.Simulate(kTestFixedTimeStep);
	//}

	//const Racecar::Real negativeUpEngineVelocity(engineMass.GetAngularVelocity());
	//const Racecar::Real negativeUpWheelVelocity(wheelMass.GetAngularVelocity());
	//if (fabs(negativeUpEngineVelocity) > kTestEpsilon || fabs(negativeUpWheelVelocity) > kTestEpsilon)
	//{
	//	log_test("Failed: 1 second constant torque, expected 1rad/sec finalVelocity.\n");
	//	return false;
	//}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

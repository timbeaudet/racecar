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
#include "linear_motion_test.h"

#include "../source/racecar.h"
#include "../source/racecar_controller.h"
#include "../source/racecar_wheel.h"
#include "../source/racecar_locked_differential.h"

#include <cstdio>

const Racecar::Real Racecar::UnitTests::kTestElipson(0.00001);
using Racecar::Real;
using Racecar::UnitTests::kTestElipson;

bool ConstructionTest(void);
bool ConstantTorqueTest(void);
bool GearReductionTest(void);

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::PerformBasicTests(void)
{
	log_test("\n\n...................\nWelcome to scrutineering\nIt is time to check over the components of your racecar...\nGood Luck!\n...................\n\n");

	bool failedTest(false);
	perform_test(ConstructionTest(), "Constructing a Rotating Body");
	perform_test(ConstantTorqueTest(), "Applying Constant Torque");
	//perform_test(GearReductionTest(), "Constant Torque through Gear Reduction");
	perform_test(WheelWithLinearMotion(), "Wheel with Linear Motion");
	perform_test(RacecarWithLinearMotion(), "Racecar with Linear Motion");
	perform_test(SpinningWheelsReleasedFromJack(), "Spinning Wheels Released From Jack");
	perform_test(FlyingCarHitsTrack(), "Flying Car Hits Track");

	perform_test(BasicEngineTest(), "Basic Engine Test");
	perform_test(WheelBrakingTest(), "Wheel Braking Test");
	perform_test(WheelAndAxleBrakingTest(), "Wheel And Axle Braking Test");	
	perform_test(WheelClutchAndEngineBrakingTest(), "Wheel Clutch And Engine Braking Test"); //Looking for potential NaN
	perform_test(EngineClutchWheelThrottleTest(), "Engine, Clutch Wheel Throttle Test");     //Checking to ensure the clutch/wheel don't spin faster than engine.
	perform_test(EngineClutchWheelBrakingTest(), "Engine, Clutch Wheel Braking Test");       //Checking if brakes slow engine with clutch disengaged.
	perform_test(EngineClutchWheelMismatchTest(), "Engine, Clutch Wheel Mismatch Test");     //Ensures the wheel and clutch remain same speeds while trying to match engine speed.
	perform_test(EngineWithConnectionTest(), "Engine With Connection Test");
	perform_test(ClutchInputTest(), "Clutch Input Test");
	perform_test(SlippingClutchTest(), "Slipping Clutch Test");
	perform_test(LockedDifferentialTest(), "Locked Differential Test");
	perform_test(LockedDifferentialBrakingTest(), "Locked Differential Braking Test");

	if (false == failedTest)
	{
		log_test("Your racecar has successfully passed technical inspection.\nYou may now go racing!\n\n");
	}
	else
	{
		log_test("Your racecar has not passed technical inspection.\nYou must fix it proper before racing!\n\n");
	}

	return (false == failedTest);
}

//--------------------------------------------------------------------------------------------------------------------//

bool ConstructionTest(void)
{
	Racecar::RotatingBody rotatingMass(10.0); //10kg-m^2.
	if (fabs(rotatingMass.GetInertia() - 10.0) > kTestElipson)
	{
		log_test("Failed accessing expected moments of inertia.");
		return false;
	}

	Racecar::RotatingBody rotatingMass2(Racecar::ComputeInertiaMetric(10.0, 1.0));
	if (fabs(rotatingMass.GetInertia() - rotatingMass2.GetInertia()) > kTestElipson)
	{
		log_test("Failed accessing expected moments of inertia.");
		return false;
	}

	const Real startingVelocity(rotatingMass.GetAngularVelocity());
	if (fabs(startingVelocity) > kTestElipson)
	{
		log_test("Failed: Expected rotating body to have no angular velocity upon construction. Found: %f\n", startingVelocity);
		return false;
	}

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
		wheel.ApplyDownstreamAngularImpulse(200.0 * 0.01, wheel); //200nm torque
		wheel.Simulate(racecarController, 0.01);
	}

	const Real expectedDownVelocity(400.0);
	const Real finalDownVelocity(wheel.GetAngularVelocity());
	if (fabs(finalDownVelocity - expectedDownVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant downstream torque, expected 400 rad/sec finalVelocity.\n");
		return false;
	}
	
	//Simulate 1 second of applying a negative 200Nm torque to the rotating body, to stop the wheel.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyDownstreamAngularImpulse(-200.0 * 0.01, wheel);
		wheel.Simulate(racecarController, 0.01);
	}

	const Real negativeDownVelocity(wheel.GetAngularVelocity());
	if (fabs(negativeDownVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant downstream torque, expected 400 rad/sec finalVelocity.\n");
		return false;
	}


	//Simulate 1 second of applying a 200Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyUpstreamAngularImpulse(200.0 * 0.01, wheel);
		wheel.Simulate(racecarController, 0.01);
	}

	const Real expectedUpVelocity(400.0);
	const Real finalUpVelocity(wheel.GetAngularVelocity());
	if (fabs(finalUpVelocity - expectedUpVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant upstream torque, expected 400 rad/sec finalVelocity.\n");
		return false;
	}

	//Simulate 1 second of applying a negative 10Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyUpstreamAngularImpulse(-200.0 * 0.01, wheel);
		wheel.Simulate(racecarController, 0.01);
	}

	const Real negativeUpVelocity(wheel.GetAngularVelocity());
	if (fabs(negativeUpVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant upstream torque, expected 400 rad/sec finalVelocity.\n");
		return false;
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool GearReductionTest(void)
{
	const Real kGearReduction(4.0);

	Racecar::DoNothingController racecarController;
	Racecar::RotatingBody engineMass(10.0); //10kg-m^2.
	Racecar::RotatingBody wheelMass(10.0);
	Racecar::LockedDifferential lockedDifferential(10.0, kGearReduction); //4:1 ratio with inertia of 10kg-m^2

	engineMass.AddOutputSource(&lockedDifferential);
	lockedDifferential.SetInputSource(&engineMass);
	lockedDifferential.AddOutputSource(&wheelMass);
	wheelMass.SetInputSource(&lockedDifferential);

	//Simulate 1 second of applying a 30Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		engineMass.ApplyDownstreamAngularImpulse(30.0 * 0.01, engineMass);
		engineMass.Simulate(0.01);
		lockedDifferential.Simulate(racecarController, 0.01);
		wheelMass.Simulate(0.01);
	}

	const Real expectedDownEngineVelocity(1.0);
	const Real finalDownEngineVelocity(engineMass.GetAngularVelocity());
	const Real expectedDownWheelVelocity(expectedDownEngineVelocity / kGearReduction);
	const Real finalDownWheelVelocity(wheelMass.GetAngularVelocity());
	if (fabs(finalDownWheelVelocity - expectedDownWheelVelocity) > kTestElipson ||
		fabs(finalDownEngineVelocity - expectedDownEngineVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant torque, expected 1rad/sec finalVelocity.\n");
		return false;
	}

	//Simulate 1 second of applying a 30Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		engineMass.ApplyDownstreamAngularImpulse(-30.0 * 0.01, engineMass);
		engineMass.Simulate(0.01); //Simulates 10ms of action.
		lockedDifferential.Simulate(racecarController, 0.01);
		wheelMass.Simulate(0.01);
	}

	const Racecar::Real negativeDownEngineVelocity(engineMass.GetAngularVelocity());
	const Racecar::Real negativeDownWheelVelocity(wheelMass.GetAngularVelocity());
	if (fabs(negativeDownEngineVelocity) > kTestElipson || fabs(negativeDownWheelVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant torque, expected 1rad/sec finalVelocity.\n");
		return false;
	}

	///////////

	//Simulate 1 second of applying a 30Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheelMass.ApplyUpstreamAngularImpulse(30.0 * 0.01, wheelMass);
		engineMass.Simulate(0.01);
		lockedDifferential.Simulate(racecarController, 0.01);
		wheelMass.Simulate(0.01);
	}

	const Real expectedUpEngineVelocity(1.0);
	const Real finalUpEngineVelocity(engineMass.GetAngularVelocity());
	const Real expectedUpWheelVelocity(expectedUpEngineVelocity / kGearReduction);
	const Real finalUpWheelVelocity(wheelMass.GetAngularVelocity());
	if (fabs(finalUpWheelVelocity - expectedUpWheelVelocity) > kTestElipson ||
		fabs(finalUpEngineVelocity - expectedUpEngineVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant torque, expected 1rad/sec finalVelocity.\n");
		return false;
	}

	//Simulate 1 second of applying a 30Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheelMass.ApplyUpstreamAngularImpulse(-30.0 * 0.01, wheelMass);
		engineMass.Simulate(0.01); //Simulates 10ms of action.
		lockedDifferential.Simulate(racecarController, 0.01);
		wheelMass.Simulate(0.01);
	}

	const Racecar::Real negativeUpEngineVelocity(engineMass.GetAngularVelocity());
	const Racecar::Real negativeUpWheelVelocity(wheelMass.GetAngularVelocity());
	if (fabs(negativeUpEngineVelocity) > kTestElipson || fabs(negativeUpWheelVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant torque, expected 1rad/sec finalVelocity.\n");
		return false;
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

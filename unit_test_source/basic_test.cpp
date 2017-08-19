///
/// @file
/// @details Defines the functions that perform the basic tests for the racecar drive-train.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "basic_test.h"
#include "test_kit.h"

#include "../player_racecar_controller.h"
#include "../racecar/racecar.h"
#include "../racecar/racecar_wheel.h"
#include "../racecar/racecar_locked_differential.h"

#include <cstdio>

const Racecar::Real Racecar::UnitTests::kTestElipson(0.00001f);
using Racecar::Real;
using Racecar::UnitTests::kTestElipson;

bool ConstructionTest(void);
bool ConstantTorqueTest(void);
bool GearReductionTest(void);

bool Racecar::UnitTests::PerformBasicTests(void)
{
	log_test("\n\n...................\nWelcome to scrutineering\nIt is time to check over the components of your racecar...\nGood Luck!\n...................\n\n");

	bool failedTest(false);
	perform_test(ConstructionTest(), "Constructing a Rotating Body");
	perform_test(ConstantTorqueTest(), "Applying Constant Torque");
	perform_test(GearReductionTest(), "Constant Torque through Gear Reduction");

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

bool ConstantTorqueTest(void)
{
	Racecar::DoNothingController racecarController;
	Racecar::Wheel wheel(8.0, 0.25); //0.5kg-m^2.

	//Simulate 1 second of applying a 200Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyDownstreamTorque(200.0, wheel); //200nm torque
		wheel.Simulate(racecarController); //Simulates 10ms of action.
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
		wheel.ApplyDownstreamTorque(-200.0f, wheel);
		wheel.Simulate(racecarController); //Simulates 10ms of action.
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
		wheel.ApplyUpstreamTorque(200.0f, wheel);
		wheel.Simulate(racecarController); //Simulates 10ms of action.
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
		wheel.ApplyUpstreamTorque(-200.0, wheel);
		wheel.Simulate(racecarController); //Simulates 10ms of action.
	}

	const Real negativeUpVelocity(wheel.GetAngularVelocity());
	if (fabs(negativeUpVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant upstream torque, expected 400 rad/sec finalVelocity.\n");
		return false;
	}

	return true;
}

bool GearReductionTest(void)
{
	const Real kGearReduction(4.0);

	Racecar::DoNothingController racecarController;
	Racecar::RotatingBody engineMass(10.0); //10kg-m^2.
	Racecar::RotatingBody wheelMass(10.0);
	Racecar::LockedDifferential lockedDifferential(kGearReduction, 10.0); //4:1 ratio with inertia of 10kg-m^2

	engineMass.AddOutputSource(&lockedDifferential);
	lockedDifferential.SetInputSource(&engineMass);
	lockedDifferential.AddOutputSource(&wheelMass);
	wheelMass.SetInputSource(&lockedDifferential);

	//Simulate 1 second of applying a 30Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		engineMass.ApplyDownstreamTorque(30.0f, engineMass);
		engineMass.Simulate(); //Simulates 10ms of action.
		lockedDifferential.Simulate(racecarController);
		wheelMass.Simulate();
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
		engineMass.ApplyDownstreamTorque(-30.0f, engineMass);
		engineMass.Simulate(); //Simulates 10ms of action.
		lockedDifferential.Simulate(racecarController);
		wheelMass.Simulate();
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
		wheelMass.ApplyUpstreamTorque(30.0f, wheelMass);
		engineMass.Simulate(); //Simulates 10ms of action.
		lockedDifferential.Simulate(racecarController);
		wheelMass.Simulate();
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
		wheelMass.ApplyUpstreamTorque(-30.0f, wheelMass);
		engineMass.Simulate(); //Simulates 10ms of action.
		lockedDifferential.Simulate(racecarController);
		wheelMass.Simulate();
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
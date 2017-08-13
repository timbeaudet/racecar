///
/// @file
/// @details Defines the functions that perform the basic tests for the racecar drivetrain.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "basic_test.h"

#include "../player_racecar_controller.h"
#include "../racecar/racecar_wheel.h"
#include "../racecar/racecar_locked_differential.h"

#include "../turtle_brains/math/tb_constants.h"

#include <cstdio>

#define log_test(message, ...)   printf(message, ##__VA_ARGS__)
static const float kTestElipson(0.0001f);

class RotatingMass : public Racecar::RotatingBody
{
public:
	explicit RotatingMass(const float momentOfInertia) :
		RotatingBody(momentOfInertia)
	{
		SetInertia(momentOfInertia);
	}

	RotatingMass(const float massInKilograms, const float radiusInMeters) :
		RotatingBody(Racecar::ComputeInertiaMetric(massInKilograms, radiusInMeters))
	{
	}

	virtual ~RotatingMass(void)
	{
	}
};



bool ConstructionTest(void);
bool ConstantTorqueTest(void);

bool GearReductionTest(void);

bool Racecar::UnitTests::PerformBasicTests(void)
{
	if (false == ConstructionTest()) { log_test("Failed: Expected to pass the ConstructionTest().\n"); return false; }
	if (false == ConstantTorqueTest()) { log_test("Failed: Expected to pass the ConstantTorqueTest().\n"); return false; }


	if (false == GearReductionTest()) { log_test("Failed: Expected to pass the GearReductionTest().\n"); return false; }

	log_test("Your racecar has successfully passed technical inspection. You may now go racing!");
	return true;
}

bool ConstructionTest(void)
{
	RotatingMass rotatingMass(10.0f); //10kg-m^2.
	if (fabsf(rotatingMass.GetInertia() - 10.0f) > kTestElipson)
	{
		log_test("Failed accessing expected moments of inertia.");
		return false;
	}

	RotatingMass rotatingMass2(10.0f, 1.0f);
	if (fabsf(rotatingMass.GetInertia() - rotatingMass2.GetInertia()) > kTestElipson)
	{
		log_test("Failed accessing expected moments of inertia.");
		return false;
	}

	const float startingVelocity(rotatingMass.GetAngularVelocity());
	if (fabsf(startingVelocity) > kTestElipson)
	{
		log_test("Failed: Expected rotating body to have no angular velocity upon construction. Found: %f\n", startingVelocity);
		return false;
	}

	return true;
}

bool ConstantTorqueTest(void)
{
	Racecar::DoNothingController racecarController;
	Racecar::Wheel wheel(10.0f); //10kg-m^2.

	//Simulate 1 second of applying a 10Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyDownstreamTorque(10.0f, wheel);
		wheel.Simulate(racecarController); //Simulates 10ms of action.
	}

	const float expectedDownVelocity(tbMath::Convert::RadiansToDegrees(1.0f));
	const float finalDownVelocity(wheel.GetAngularVelocity());
	if (fabsf(finalDownVelocity - expectedDownVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant downstream torque, expected 1rad/sec finalVelocity.\n");
		return false;
	}
	
	//Simulate 1 second of applying a negative 10Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyDownstreamTorque(-10.0f, wheel);
		wheel.Simulate(racecarController); //Simulates 10ms of action.
	}

	const float negativeDownVelocity(wheel.GetAngularVelocity());
	if (fabsf(negativeDownVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant downstream torque, expected 1rad/sec finalVelocity.\n");
		return false;
	}


	//Simulate 1 second of applying a 10Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyUpstreamTorque(10.0f, wheel);
		wheel.Simulate(racecarController); //Simulates 10ms of action.
	}

	const float expectedUpVelocity(tbMath::Convert::RadiansToDegrees(1.0f));
	const float finalUpVelocity(wheel.GetAngularVelocity());
	if (fabsf(finalUpVelocity - expectedUpVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant upstream torque, expected 1rad/sec finalVelocity.\n");
		return false;
	}

	//Simulate 1 second of applying a negative 10Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyUpstreamTorque(-10.0f, wheel);
		wheel.Simulate(racecarController); //Simulates 10ms of action.
	}

	const float negativeUpVelocity(wheel.GetAngularVelocity());
	if (fabsf(negativeUpVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant upstream torque, expected 1rad/sec finalVelocity.\n");
		return false;
	}

	return true;
}

bool GearReductionTest(void)
{
	const float kGearReduction(4.0f);

	Racecar::DoNothingController racecarController;
	Racecar::Wheel engineMass(10.0f); //10kg-m^2.
	Racecar::Wheel wheelMass(10.0f);
	Racecar::LockedDifferential lockedDifferential(kGearReduction, 10.0f); //4:1 ratio with inertia of 10kg-m^2

	engineMass.AddOutputSource(&lockedDifferential);
	lockedDifferential.SetInputSource(&engineMass);
	lockedDifferential.AddOutputSource(&wheelMass);
	wheelMass.SetInputSource(&lockedDifferential);

	//Simulate 1 second of applying a 30Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheelMass.ApplyUpstreamTorque(30.0f, wheelMass);
		engineMass.Simulate(racecarController); //Simulates 10ms of action.
		lockedDifferential.Simulate(racecarController);
		wheelMass.Simulate(racecarController);
	}

	const float expectedEngineVelocity(tbMath::Convert::RadiansToDegrees(1.0f));
	const float finalEngineVelocity(engineMass.GetAngularVelocity());
	const float expectedWheelVelocity(expectedEngineVelocity / kGearReduction);
	const float finalWheelVelocity(wheelMass.GetAngularVelocity());

	if (fabsf(finalWheelVelocity - expectedWheelVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant torque, expected 1rad/sec finalVelocity.\n");
		return false;
	}

	//Simulate 1 second of applying a 30Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheelMass.ApplyUpstreamTorque(-30.0f, wheelMass);
		engineMass.Simulate(racecarController); //Simulates 10ms of action.
		lockedDifferential.Simulate(racecarController);
		wheelMass.Simulate(racecarController);
	}

	const float nowFinalEngineVelocity(engineMass.GetAngularVelocity());
	const float nowFinalWheelVelocity(wheelMass.GetAngularVelocity());
	if (fabsf(nowFinalEngineVelocity) > kTestElipson || fabsf(nowFinalWheelVelocity) > kTestElipson)
	{
		log_test("Failed: 1 second constant torque, expected 1rad/sec finalVelocity.\n");
		return false;
	}

	return true;
}
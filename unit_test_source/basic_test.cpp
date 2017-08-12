///
/// @file
/// @details Defines the functions that perform the basic tests for the racecar drivetrain.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "basic_test.h"

#include "../racecar/racecar_wheel.h"

#include <cstdio>

#define log_test(message, ...)   printf(message, ##__VA_ARGS__)
static const float kTestElipson(0.0001f);

class RotatingMass : public Racecar::RotatingBody
{
public:
	explicit RotatingMass(const float momentOfInertia) :
		RotatingBody()
	{
		SetInertia(momentOfInertia);
	}

	RotatingMass(const float massInKilograms, const float radiusInMeters) :
		RotatingBody()
	{
		SetInertia(Racecar::ComputeInertiaMetric(massInKilograms, radiusInMeters));
	}

	virtual ~RotatingMass(void)
	{
	}
};

bool ConstructionTest(void);
bool ConstantTorqueTest(void);

bool Racecar::UnitTests::PerformBasicTests(void)
{
	if (false == ConstructionTest()) { log_test("Failed: Expected to pass the ConstructionTest().\n"); return false; }
	if (false == ConstantTorqueTest()) { log_test("Failed: Expected to pass the ConstantTorqueTest().\n"); return false; }

	log_test("The Racecar has successfully passed technical inspection. You may now go racing!");
	return true;
}


bool ConstructionTest(void)
{
	RotatingMass rotatingMass(10.0f); //10kg-m.
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
	RotatingMass rotatingMass(10.0f); //10kg-m.

	//Simulate 1 second of applying a 10Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		rotatingMass.ApplyDownstreamTorque(10.0f, rotatingMass);
		rotatingMass.Simulate(); //Simulates 10ms of action.
	}

	const float finalVelocity(rotatingMass.GetAngularVelocity());
	log_test("Final velocity: %f\n", finalVelocity);
	
	return false;
}
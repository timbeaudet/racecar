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

bool Racecar::UnitTests::PerformBasicTests(void)
{
	RotatingMass rotatingMass(10.0f); //10kg-m.
	RotatingMass rotatingMass2(10.0f, 1.0f);

	if (fabsf(rotatingMass.GetInertia() - rotatingMass2.GetInertia() > kTestElipson))
	{
		log_test("Failed accessing expected moments of inertia.");
		return false;
	}

	return false;
}

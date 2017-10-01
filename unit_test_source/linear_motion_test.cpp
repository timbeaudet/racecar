///
/// @file
/// @details Defines the functions that perform the basic tests for the racecar drive-train.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "linear_motion_test.h"
#include "test_kit.h"

#include "../player_racecar_controller.h"
#include "../racecar/racecar.h"
#include "../racecar/racecar_body.h"
#include "../racecar/racecar_wheel.h"
#include "../racecar/racecar_locked_differential.h"

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::WheelWithLinearMotion(void)
{
	// This test is all about taking a wheel that is on the ground, from a stopped position and applying a torque to the
	// wheel to convert angular motion to linear motion. - Assuming infinite friction, no resistances.

	Racecar::DoNothingController racecarController;
	Racecar::Wheel wheel(8.0, 0.25);
	wheel.SetOnGround(true, Racecar::Wheel::kInfiniteFriction);

	//Simulate 1 second of applying a 200Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyDownstreamTorque(200.0, wheel); //200nm torque
		wheel.Simulate(racecarController); //Simulates 10ms of action.
	}

	const Real expectedLinearVelocity(100.0);  //100m/s
	const Real& finalLinearVelocity(wheel.GetLinearVelocity());
	if (fabs(finalLinearVelocity - expectedLinearVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}
	
	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::RacecarWithLinearMotion(void)
{
	// This test is all about taking a racecar on the ground, from a stopped position and applying a torque to the drive
	// wheel of the car to convert angular motion to linear motion on both the wheel and the car body. - Assuming infinite
	// friction, no resistances.

	Racecar::DoNothingController racecarController;
	Racecar::RacecarBody carBody(92.0);
	Racecar::Wheel wheel(8.0, 0.25);
	carBody.SetWheel(0, &wheel);
	wheel.SetRacecarBody(&carBody);
	wheel.SetOnGround(true, Racecar::Wheel::kInfiniteFriction);

	//Simulate 1 second of applying a 200Nm torque to the rotating body.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyDownstreamTorque(200.0, wheel); //200nm torque
		wheel.Simulate(racecarController); //Simulates 10ms of action.
		carBody.Simulate(racecarController);
	}

	const Real expectedLinearVelocity(8.0);  //m/s
	const Real& finalLinearVelocity(wheel.GetLinearVelocity());
	if (fabs(finalLinearVelocity - expectedLinearVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::SpinningWheelOnGround(void)
{
	Racecar::DoNothingController racecarController;
	Racecar::RacecarBody carBody(92.0);
	Racecar::Wheel wheel(8.0, 0.25);
	carBody.SetWheel(0, &wheel);
	wheel.SetRacecarBody(&carBody);
	wheel.SetOnGround(true, 0.05); //0.05 for ice friction, 0.7 for pavement friction.

	carBody.SetLinearVelocity(0.0);
	wheel.SetLinearVelocity(0.0);
	wheel.SetAngularVelocity(40.0);

	for (int timer(0); timer < 10; timer += 10)
	{
		wheel.Simulate(racecarController);
		carBody.Simulate(racecarController);
	}

//	Real carVelocity(carBody.GetLinearVelocity());
//	Real wheelAngVelocity(wheel.GetAngularVelocity());

	//
	//

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

struct TestBlob
{
	Racecar::Real mFrictionCoefficient;
	Racecar::Real mExpectedLinearVelocity[2];  //After a single step, after all steps.
	Racecar::Real mExpectedAngularVelocity[2]; //After a single step, after all steps.
	int mTestTime;
};

bool Racecar::UnitTests::SpinningWheelsReleasedFromJack(void)
{
	//Driver comes into a pitstop, takes all four tires. The fronts and reards are on and off, and the front jack drops.
	//The lollipop guy signals to GO, but the rear jack is stuck - rear tires hanging in the air. Driver floors the throttle
	//rear wheels spin up to 40rad/s (approx 20-25mph). Finally the rear jack is freed and the car drops to ground with
	//already spinning tires.

	std::array<TestBlob, 3> tests{
		TestBlob{ -1.0, { 0.740740740740, 0.740740740740 }, { 2.962962962962, 2.962962962962 }, 10 },   //infinite
		TestBlob{ 0.05, { 0.005,          0.740740740740 }, { 39.75,          2.962962962962 }, 1000 }, //ice
		TestBlob{ 0.70, { 0.07,           0.740740740740 }, { 36.5,           2.962962962962 }, 1000 }  //pavement
	};

	for (const TestBlob& test : tests)
	{
		Racecar::DoNothingController racecarController;
		Racecar::RacecarBody carBody(92.0);
		carBody.SetLinearVelocity(0.0);

		Racecar::Wheel wheel(8.0, 0.25);
		wheel.SetLinearVelocity(0.0);
		wheel.SetAngularVelocity(40.0);

		carBody.SetWheel(0, &wheel);
		wheel.SetRacecarBody(&carBody);
		wheel.SetOnGround(true, test.mFrictionCoefficient); //0.05 for ice friction, 0.7 for pavement friction.

		//Simulate a time step, with infinite friction this should be all that is needed.
		wheel.Simulate(racecarController); //Simulates 10ms of action.
		carBody.Simulate(racecarController);

		{	//Check for results after a single time step, should be matched for infinite friction.
			if (fabs(wheel.GetLinearVelocity() - carBody.GetLinearVelocity()) > Racecar::UnitTests::kTestElipson)
			{	//The wheel and car body should move linearly at the same rate!
				return false;
			}

			if (fabs(wheel.GetLinearVelocity() - test.mExpectedLinearVelocity[0]) > Racecar::UnitTests::kTestElipson)
			{
				return false;
			}

			if (fabs(wheel.GetAngularVelocity() - test.mExpectedAngularVelocity[0]) > Racecar::UnitTests::kTestElipson)
			{
				return false;
			}
		}

		for (int timer(10); timer < test.mTestTime; timer += 10)
		{
			wheel.Simulate(racecarController); //Simulates 10ms of action.
			carBody.Simulate(racecarController);
		}

		{	//Check for results after all time steps.
			if (fabs(wheel.GetLinearVelocity() - carBody.GetLinearVelocity()) > Racecar::UnitTests::kTestElipson)
			{	//The wheel and car body should move linearly at the same rate!
				return false;
			}

			if (fabs(wheel.GetLinearVelocity() - test.mExpectedLinearVelocity[1]) > Racecar::UnitTests::kTestElipson)
			{
				return false;
			}

			if (fabs(wheel.GetAngularVelocity() - test.mExpectedAngularVelocity[1]) > Racecar::UnitTests::kTestElipson)
			{
				return false;
			}
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::FlyingCarHitsTrack(void)
{
	const Real initialVelocity(40.0); //m/s

	Racecar::DoNothingController racecarController;
	Racecar::RacecarBody racecarBody(92.0);
	Racecar::Wheel wheel(8.0, 0.25);
	racecarBody.SetWheel(0, &wheel);
	racecarBody.SetLinearVelocity(initialVelocity); //m/s
	wheel.SetRacecarBody(&racecarBody);
	wheel.SetLinearVelocity(racecarBody.GetLinearVelocity());

	//Simulate 1 second of applying a no torques.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.Simulate(racecarController); //Simulates 10ms of action.
		racecarBody.Simulate(racecarController);
	}

	const Real& finalLinearVelocity(wheel.GetLinearVelocity());
	if (fabs(finalLinearVelocity - initialVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	if (fabs(wheel.GetAngularVelocity()) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	//Still flying in the air, no change in angular or linear velocities.
	wheel.SetOnGround(true, Racecar::Wheel::kInfiniteFriction); //Land on the ground.
	wheel.Simulate(racecarController); //Simulates 10ms of action.
	racecarBody.Simulate(racecarController);

	const Real expectedLinearVelocity(37.037037037);
	const Real expectedAngularVelocity(148.148148148);
	const Real finalRacecarVelocity(racecarBody.GetLinearVelocity());
	const Real finalWheelLinearVelocity(wheel.GetLinearVelocity());
	const Real finalWheelAngularVelocity(wheel.GetAngularVelocity());
	if (fabs(expectedLinearVelocity - finalRacecarVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	if (fabs(expectedLinearVelocity - finalWheelLinearVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	if (fabs(expectedAngularVelocity - finalWheelAngularVelocity) > Racecar::UnitTests::kTestElipson)
	{
		return false;
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//


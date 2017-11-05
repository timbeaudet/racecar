///
/// @file
/// @details Defines the functions that perform the basic tests for the racecar drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "linear_motion_test.h"
#include "test_kit.h"

#include "../source/racecar.h"
#include "../source/racecar_controller.h"
#include "../source/racecar_body.h"
#include "../source/racecar_wheel.h"

#include <string>
#include <fstream>

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
		wheel.ApplyDownstreamAngularImpulse(200.0 * 0.01); //200nm torque
		wheel.Simulate(racecarController, kTestFixedTimeStep);
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
		wheel.ApplyDownstreamAngularImpulse(200.0 * 0.01); //200nm torque
		wheel.Simulate(racecarController, kTestFixedTimeStep);
		carBody.Simulate(racecarController, kTestFixedTimeStep);
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

struct FrictionTestBlob
{
	std::string mTestName;
	Racecar::Real mFrictionCoefficient;
	Racecar::Real mExpectedLinearVelocity[2];  //After a single step, after all steps.
	Racecar::Real mExpectedAngularVelocity[2]; //After a single step, after all steps.
	int mTestTime;
};

bool Racecar::UnitTests::SpinningWheelsReleasedFromJack(void)
{
	//Driver comes into a pit-stop, takes all four tires. The fronts and rears are on and off, and the front jack drops.
	//The lollipop guy signals to GO, but the rear jack is stuck - rear tires hanging in the air. Driver floors the throttle
	//rear wheels spin up to 40rad/s (approx 20-25mph). Finally the rear jack is freed and the car drops to ground with
	//already spinning tires.

	std::array<FrictionTestBlob, 3> tests{
		FrictionTestBlob{ "Infinite Friction", -1.0, { 0.740740740740, 0.740740740740 }, { 2.962962962962, 2.962962962962 }, 10 },
		FrictionTestBlob{ "Ice Friction",      0.05, { 0.005,          0.740740740740 }, { 39.75,          2.962962962962 }, 2000 },
		FrictionTestBlob{ "Pavement Friction", 0.70, { 0.07,           0.740740740740 }, { 36.5,           2.962962962962 }, 1000 }
	};

	std::ofstream outFile("data/outputs/jack_released.txt");

	for (const FrictionTestBlob& test : tests)
	{
		outFile << test.mTestName << std::endl;

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
		wheel.Simulate(racecarController, kTestFixedTimeStep); //Simulates 10ms of action.
		carBody.Simulate(racecarController, kTestFixedTimeStep);

		outFile << "10\t" << wheel.GetLinearVelocity() << "\t" << wheel.GetAngularVelocity() << "\n";
		outFile.flush();

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

		//Simulate the time steps for the remainder of the test, when complete the car/wheel should be in equilibrium from friction.
		for (int timer(10); timer < test.mTestTime; timer += 10)
		{
			wheel.Simulate(racecarController, kTestFixedTimeStep); //Simulates 10ms of action.
			carBody.Simulate(racecarController, kTestFixedTimeStep);

			outFile << timer + 10 << "\t" << wheel.GetLinearVelocity() << "\t" << wheel.GetAngularVelocity() << "\n";
			outFile.flush();
		}

		outFile << "\nExpected: " << test.mTestTime << "\t" << test.mExpectedLinearVelocity[1] << "\t" << test.mExpectedAngularVelocity[1] << "\n";
		outFile << "\n\n\n";
		outFile.flush();

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
	//Driver is flying through the air in his rally car, he has hit the brakes to stop the wheels from spinning, though
	//the car is still flying through the air at ~90mph (40m/s) and when it lands on the ground the wheels shall start
	//spinning while taking some of the momentum away from the car.

	std::array<FrictionTestBlob, 3> tests{
		FrictionTestBlob{ "Infinite Friction", -1.0,{ 37.037037037, 37.037037037 },{ 148.148148148, 148.148148148 }, 10 },
		FrictionTestBlob{ "Ice Friction",      0.05,{ 39.995,          37.037037037 },{ 0.25,          148.148148148 }, 10000 },
		FrictionTestBlob{ "Pavement Friction", 0.70,{ 39.93,           37.037037037 },{ 3.5,           148.148148148 }, 1000 }
	};

	const Real initialLinearVelocity(40); //m/s

	for (const FrictionTestBlob test : tests)
	{
		Racecar::DoNothingController racecarController;
		Racecar::RacecarBody racecarBody(92.0); //kg
		racecarBody.SetLinearVelocity(initialLinearVelocity); //m/s

		Racecar::Wheel wheel(8.0, 0.25); //kg,  radius in meters
		wheel.SetLinearVelocity(racecarBody.GetLinearVelocity());
		wheel.SetAngularVelocity(0.0);

		racecarBody.SetWheel(0, &wheel);
		wheel.SetRacecarBody(&racecarBody);
		wheel.SetOnGround(false, test.mFrictionCoefficient);

		//Simulate 500ms of the car flying through the air, the car has NOT landed on the ground, and should have identical state.
		for (int timer(10); timer < 500; timer += 10)
		{
			wheel.Simulate(racecarController, kTestFixedTimeStep); //Simulates 10ms of action.
			racecarBody.Simulate(racecarController, kTestFixedTimeStep);
		}

		{	//Check that the car and wheel are in identical states as when we started the test, (haven't touched ground yet!)
			const Real& finalLinearVelocity(wheel.GetLinearVelocity());
			if (fabs(finalLinearVelocity - initialLinearVelocity) > Racecar::UnitTests::kTestElipson)
			{
				return false;
			}

			if (fabs(wheel.GetAngularVelocity()) > Racecar::UnitTests::kTestElipson)
			{
				return false;
			}
		}

		//Car has now landed on the ground, simulate a single time step, with infinite friction this should be all that is needed.
		wheel.SetOnGround(true, test.mFrictionCoefficient);
		wheel.Simulate(racecarController, 0.01); //Simulates 10ms of action.
		racecarBody.Simulate(racecarController, 0.01);

		{	//Check for results after a single time step, should be matched for infinite friction.
			if (fabs(wheel.GetLinearVelocity() - racecarBody.GetLinearVelocity()) > Racecar::UnitTests::kTestElipson)
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

		//Simulate the time steps for the remainder of the test, when complete the car/wheel should be in equilibrium from friction.
		for (int timer(10); timer < test.mTestTime; timer += 10)
		{
			wheel.Simulate(racecarController, kTestFixedTimeStep); //Simulates 10ms of action.
			racecarBody.Simulate(racecarController, kTestFixedTimeStep);
		}

		{	//Check for results after a single time step, should be matched for infinite friction.
			if (fabs(wheel.GetLinearVelocity() - racecarBody.GetLinearVelocity()) > Racecar::UnitTests::kTestElipson)
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


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
	Racecar::DoNothingController racecarController;
	Racecar::Wheel wheel(8.0, 0.25);
	wheel.SetOnGround(true, Racecar::Wheel::kInfiniteFriction);

	//Simulate 1 second of applying a 200Nm torque to the wheel.
	for (int timer(0); timer < 1000; timer += 10)
	{
		wheel.ApplyDownstreamAngularImpulse(200.0 * 0.01); //200nm torque

		wheel.ControllerChange(racecarController);
		wheel.Simulate(kTestFixedTimeStep);
	}

	const Real expectedLinearVelocity(100.0);  //100m/s
	const Real expectedAngularVelocity(expectedLinearVelocity / wheel.GetRadius());
	const Real& finalLinearVelocity(wheel.GetLinearVelocity());
	if (fabs(finalLinearVelocity - expectedLinearVelocity) > Racecar::UnitTests::kTestEpsilon)
	{
		return false;
	}
	if (fabs(wheel.GetAngularVelocity() - expectedAngularVelocity) > kTestEpsilon)
	{
		return false;
	}
	
	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

struct RacecarLinearMotionBlob
{
	Racecar::Real mWheelMass;      //kg
	Racecar::Real mWheelRadius;    //m
	Racecar::Real mRacecarMass;    //kg
	Racecar::Real mConstantTorque; //Nm

	size_t mSimulatedSteps;        //Each step is 10ms.
	Racecar::Real mExpectedAngularVelocity;
	Racecar::Real mExpectedLinearVelocity;
};

bool Racecar::UnitTests::RacecarWithLinearMotion(void)
{
	std::array<RacecarLinearMotionBlob, 2> tests{
		///                   wheel inertia, radius   car mass   torque   steps      exAngVel       exLinVel
		RacecarLinearMotionBlob{  8.0000,    0.2500,   92.000,     200.0,   100,      32.000,         8.0000, },
		RacecarLinearMotionBlob{ 18.1437,    0.2794,   1042.0,     110.0,   100,    1.329153517,     0.3713654926, },
	};

	for (const RacecarLinearMotionBlob& test : tests)
	{
		Racecar::DoNothingController racecarController;
		Racecar::RacecarBody carBody(test.mRacecarMass);
		Racecar::Wheel wheel(test.mWheelMass, test.mWheelRadius);
		carBody.SetWheel(0, &wheel);
		wheel.SetRacecarBody(&carBody);
		wheel.SetOnGround(true, Racecar::Wheel::kInfiniteFriction);

		for (size_t timeStep(0); timeStep < test.mSimulatedSteps; ++timeStep)
		{
			wheel.ControllerChange(racecarController);
			carBody.ControllerChange(racecarController);

			wheel.ApplyDownstreamAngularImpulse(test.mConstantTorque * 0.01);
			wheel.Simulate(kTestFixedTimeStep);
			carBody.Simulate(kTestFixedTimeStep);
		}

		if (fabs(wheel.GetLinearVelocity() - test.mExpectedLinearVelocity) > Racecar::UnitTests::kTestEpsilon)
		{
			return false;
		}
		if (fabs(wheel.GetAngularVelocity() - test.mExpectedAngularVelocity) > Racecar::UnitTests::kTestEpsilon)
		{
			return false;
		}
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

		wheel.ControllerChange(racecarController);
		carBody.ControllerChange(racecarController);
		wheel.Simulate(kTestFixedTimeStep); //Simulates 10ms of action.
		carBody.Simulate(kTestFixedTimeStep);

		outFile << "10\t" << wheel.GetLinearVelocity() << "\t" << wheel.GetAngularVelocity() << "\n";
		outFile.flush();

		{	//Check for results after a single time step, should be matched for infinite friction.
			if (fabs(wheel.GetLinearVelocity() - carBody.GetLinearVelocity()) > Racecar::UnitTests::kTestEpsilon)
			{	//The wheel and car body should move linearly at the same rate!
				return false;
			}

			if (fabs(wheel.GetLinearVelocity() - test.mExpectedLinearVelocity[0]) > Racecar::UnitTests::kTestEpsilon)
			{
				return false;
			}

			if (fabs(wheel.GetAngularVelocity() - test.mExpectedAngularVelocity[0]) > Racecar::UnitTests::kTestEpsilon)
			{
				return false;
			}
		}

		//Simulate the time steps for the remainder of the test, when complete the car/wheel should be in equilibrium from friction.
		for (int timer(10); timer < test.mTestTime; timer += 10)
		{
			wheel.ControllerChange(racecarController);
			carBody.ControllerChange(racecarController);
			wheel.Simulate(kTestFixedTimeStep); //Simulates 10ms of action.
			carBody.Simulate(kTestFixedTimeStep);

			outFile << timer + 10 << "\t" << wheel.GetLinearVelocity() << "\t" << wheel.GetAngularVelocity() << "\n";
			outFile.flush();
		}

		outFile << "\nExpected: " << test.mTestTime << "\t" << test.mExpectedLinearVelocity[1] << "\t" << test.mExpectedAngularVelocity[1] << "\n";
		outFile << "\n\n\n";
		outFile.flush();

		{	//Check for results after all time steps.
			if (fabs(wheel.GetLinearVelocity() - carBody.GetLinearVelocity()) > Racecar::UnitTests::kTestEpsilon)
			{	//The wheel and car body should move linearly at the same rate!
				return false;
			}

			if (fabs(wheel.GetLinearVelocity() - test.mExpectedLinearVelocity[1]) > Racecar::UnitTests::kTestEpsilon)
			{
				return false;
			}

			if (fabs(wheel.GetAngularVelocity() - test.mExpectedAngularVelocity[1]) > Racecar::UnitTests::kTestEpsilon)
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
			wheel.ControllerChange(racecarController);
			racecarBody.ControllerChange(racecarController);

			wheel.Simulate(kTestFixedTimeStep); //Simulates 10ms of action.
			racecarBody.Simulate(kTestFixedTimeStep);
		}

		{	//Check that the car and wheel are in identical states as when we started the test, (haven't touched ground yet!)
			const Real& finalLinearVelocity(wheel.GetLinearVelocity());
			if (fabs(finalLinearVelocity - initialLinearVelocity) > Racecar::UnitTests::kTestEpsilon)
			{
				return false;
			}

			if (fabs(wheel.GetAngularVelocity()) > Racecar::UnitTests::kTestEpsilon)
			{
				return false;
			}
		}

		//Car has now landed on the ground, simulate a single time step, with infinite friction this should be all that is needed.
		wheel.SetOnGround(true, test.mFrictionCoefficient);

		wheel.ControllerChange(racecarController);
		racecarBody.ControllerChange(racecarController);
		wheel.Simulate(kTestFixedTimeStep); //Simulates 10ms of action.
		racecarBody.Simulate(kTestFixedTimeStep);

		{	//Check for results after a single time step, should be matched for infinite friction.
			if (fabs(wheel.GetLinearVelocity() - racecarBody.GetLinearVelocity()) > Racecar::UnitTests::kTestEpsilon)
			{	//The wheel and car body should move linearly at the same rate!
				return false;
			}

			if (fabs(wheel.GetLinearVelocity() - test.mExpectedLinearVelocity[0]) > Racecar::UnitTests::kTestEpsilon)
			{
				return false;
			}

			if (fabs(wheel.GetAngularVelocity() - test.mExpectedAngularVelocity[0]) > Racecar::UnitTests::kTestEpsilon)
			{
				return false;
			}
		}

		//Simulate the time steps for the remainder of the test, when complete the car/wheel should be in equilibrium from friction.
		for (int timer(10); timer < test.mTestTime; timer += 10)
		{
			wheel.ControllerChange(racecarController);
			racecarBody.ControllerChange(racecarController);
			wheel.Simulate(kTestFixedTimeStep); //Simulates 10ms of action.
			racecarBody.Simulate(kTestFixedTimeStep);
		}

		{	//Check for results after a single time step, should be matched for infinite friction.
			if (fabs(wheel.GetLinearVelocity() - racecarBody.GetLinearVelocity()) > Racecar::UnitTests::kTestEpsilon)
			{	//The wheel and car body should move linearly at the same rate!
				return false;
			}

			if (fabs(wheel.GetLinearVelocity() - test.mExpectedLinearVelocity[1]) > Racecar::UnitTests::kTestEpsilon)
			{
				return false;
			}

			if (fabs(wheel.GetAngularVelocity() - test.mExpectedAngularVelocity[1]) > Racecar::UnitTests::kTestEpsilon)
			{
				return false;
			}
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//


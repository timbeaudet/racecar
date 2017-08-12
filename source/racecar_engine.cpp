///
/// @file
/// @details This is a simple start to simulating a physical engine that simply has a torque table to look up.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_engine.h"
#include "racecar_controller.h"

#include "../drive_train_simulation.h" //For kFixedTime 
#include "../turtle_brains/tb_math_kit.h"
#include "../turtle_brains/tb_debug_kit.h"

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Engine::Engine(void) :
	mTorqueTable(),
	mMaximumTorque(162.0f)
{
	InitializeTorqueTableToMiata();

	SetInertia(Racecar::ComputeInertiaImperial(15.0f, 3.5f));// +ComputeInertia(9.0f, 7.87f));
	SetAngularVelocity(360.0f / 60 * 1000);
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Engine::~Engine(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Engine::InitializeTorqueTableToMiata(void)
{	//http://www.automobile-catalog.com/curve/1999/1667030/mazda_mx-5_1_9.html
	mTorqueTable[0] = 25.0f;   //500rpm
	mTorqueTable[1] = 75.0f;   //1000rpm
	mTorqueTable[2] = 112.0f;
	mTorqueTable[3] = 130.0f;  //2000rpm
	mTorqueTable[4] = 137.0f;
	mTorqueTable[5] = 150.0f;  //3000rpm
	mTorqueTable[6] = 155.0f;
	mTorqueTable[7] = 158.0f;  //4000rpm
	mTorqueTable[8] = 162.0f;
	mTorqueTable[9] = 160.0f;  //5000rpm
	mTorqueTable[10] = 159.0f;
	mTorqueTable[11] = 156.5f; //6000rpm
	mTorqueTable[12] = 151.0f;
	mTorqueTable[13] = 127.0f; //7000rpm
	mTorqueTable[14] = 25.0f;
	mTorqueTable[15] = 0.0f;  //8000rpm

	tb_error_if(kTorqueTableSize != 16, "Error: Expected table size to be 16, may need to change step size or something.");

	mMaximumTorque = 162.0f; //If unknown a search could find it...
	for (size_t index(0); index < kTorqueTableSize; ++index)
	{	//Normalize the curve so it fits in range 0 to 1, from no to max torque??
		mTorqueTable[index] /= mMaximumTorque;
	}
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::Engine::GetMaximumTorque(void) const
{
	return mMaximumTorque;
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::Engine::GetOutputTorque(const float engineSpeedRPM) const
{
	return GetOutputValue(engineSpeedRPM) * mMaximumTorque;
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::Engine::GetOutputValue(const float engineSpeedRPM) const
{
	float previous = mTorqueTable.front();
	for (size_t index(0); index < kTorqueTableSize; ++index)
	{
		float currentRPM((index) * 500.0f);
		if (engineSpeedRPM > currentRPM)
		{
			previous = mTorqueTable[index];
			continue;
		}

		//0.2 = (1500 - 1400) / 500.0f

		float percentage = 1.0f - ((currentRPM - engineSpeedRPM) / 500.0f);
		return tbMath::Interpolation::Linear(percentage, previous, mTorqueTable[index]);
	}

	return mTorqueTable.back();
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Engine::Simulate(const Racecar::RacecarControllerInterface& racecarController)
{
	const float& kFixedTime(DriveTrainSimulation::kFixedTime);
	const float revolutions(GetEngineSpeedRPM() / 60.0f * kFixedTime);

	if (GetEngineSpeedRPM() < 6500)
	{
		const float minimumIdleTorque(tbMath::Convert::FootPoundsToNewtonMeters(5.2f));
		const float onThrottleTorque(GetOutputTorque(GetEngineSpeedRPM()) * racecarController.GetThrottlePosition());
		const float appliedEngineTorque = tbMath::Maximum(minimumIdleTorque, onThrottleTorque);
		ApplyDownstreamTorque(appliedEngineTorque * revolutions, *this);
	}

	//Resistance of 1Nm for every 32 rad/s <-- THIS COMMENT MIGHT NOT BE TRUE ANYMORE...
	const float engineResistanceTorque(tbMath::Convert::DegreesToRadians(GetAngularVelocity()) * 0.0625f);
	ApplyDownstreamTorque(-engineResistanceTorque, *this);

	//Now that all torques have been applied to the engine, step it forward in time.
	RotatingBody::Simulate();

	const float differenceTo1000(GetEngineSpeedRPM() - 1000.0f);
	if (differenceTo1000 < 0.0f)
	{
		//SetAngularVelocity(Racecar::RevolutionsMinuteToDegreesSecond(1000.0f));
		const float totalInertia(ComputeDownstreamInertia(*this));
		ApplyDownstreamTorque(-differenceTo1000 * totalInertia, *this);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::Engine::GetEngineSpeedRPM(void) const
{
	//deg   60 sec   1 rev       rev
	//--- * ---    * ---         ---
	//sec   1 min    360 deg     min


	return GetAngularVelocity() * 60.0f / 360.0f;
}

//-------------------------------------------------------------------------------------------------------------------//

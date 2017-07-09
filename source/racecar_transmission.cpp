///
/// @file
/// @details This is a basic simulation for a 5 speed h-shifter transmission.
///
/// <!-- Copyright (c) 2017 Tim Beaudet - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_transmission.h"
#include "racecar_controller.h"
#include "../drive_train_simulation.h" //for kFixedStep

//--------------------------------------------------------------------------------------------------------------------//

const float kGearRatios[] = { 3.136f, 1.888f, 1.333f, 1.0f, 0.814f };
//const float kInputGearRadius[] = { 0.967118f, 1.38504f, 1.71453f, 2.0f, 2.20507f };
//const float kOutputGearRadius[] = { 3.03288f, 2.61496f, 2.28547f, 2.0f, 1.79493f };
const float kDogCollarSpots[] = { 3.5f, 5.625f, 7.75f }; //+ or - .375" to engage

//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//

float InchToMeter(float input) { return tbMath::Convert::InchesToMeters(input); }

Racecar::Transmission::Transmission(void) :
	mInputShaftSpeed(0.0f),
	mOutputShaftSpeed(0.0f)
{
	SetInertia(Racecar::ComputeInertia(10.0f, 3.0f));
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Transmission::~Transmission(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::Simulate(const RacecarControllerInterface& racecarController)
{
	tb_unused(racecarController);
	RotatingBody& inputSource(GetExpectedInputSource());

	mInputShaftSpeed = Racecar::DegreesSecondToRevolutionsMinute(inputSource.GetAngularVelocity()); //divide for visual effects.
	const float inputRotation(mInputShaftSpeed * 360.0f / 60.0f * DriveTrainSimulation::kFixedTime); //RPM to DegreesPerStep

	//if (Gear::Neutral == mCurrentGear) { mOutputShaftSpeed = 0.0f; }
	//else if (Gear::First == mCurrentGear) { mOutputShaftSpeed = mInputShaftSpeed / kGearRatios[0]; }
	//else if (Gear::Second == mCurrentGear) { mOutputShaftSpeed = mInputShaftSpeed / kGearRatios[1]; }
	//else if (Gear::Third == mCurrentGear) { mOutputShaftSpeed = mInputShaftSpeed / kGearRatios[2]; }
	//else if (Gear::Fourth == mCurrentGear) { mOutputShaftSpeed = mInputShaftSpeed / kGearRatios[3]; }
	//else if (Gear::Fifth == mCurrentGear) { mOutputShaftSpeed = mInputShaftSpeed / kGearRatios[4]; }
	
	const float outputRotation(mOutputShaftSpeed * 360.0f / 60.0f * DriveTrainSimulation::kFixedTime);

	const float finalDriveRatio(4.30f); //Should be down the line, but meh.
	SetAngularVelocity(Racecar::RevolutionsMinuteToDegreesSecond(mOutputShaftSpeed / finalDriveRatio));
}

//--------------------------------------------------------------------------------------------------------------------//

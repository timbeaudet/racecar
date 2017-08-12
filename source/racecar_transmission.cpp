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

const float kGearRatios[] = { 0.0f, -3.2f, 3.136f, 1.888f, 1.333f, 1.0f, 0.814f }; //note: N, R, 1st ... R is NOT correct miata ratio.
//const float kInputGearRadius[] = { 0.967118f, 1.38504f, 1.71453f, 2.0f, 2.20507f };
//const float kOutputGearRadius[] = { 3.03288f, 2.61496f, 2.28547f, 2.0f, 1.79493f };
const float kDogCollarSpots[] = { 3.5f, 5.625f, 7.75f }; //+ or - .375" to engage

float InchToMeter(float input) { return tbMath::Convert::InchesToMeters(input); }

constexpr float RatioForGear(const Racecar::Gear& gear)
{	//const float kGearRatios[] = { 0.0f, -3.2f, 3.136f, 1.888f, 1.333f, 1.0f, 0.814f };
	return
		(Racecar::Gear::Reverse == gear) ? -3.2f : //Note: Not the actual miata reverse gear ratio!
		(Racecar::Gear::Neutral == gear) ? 0.0f :
		(Racecar::Gear::First == gear) ? 3.136f :
		(Racecar::Gear::Second == gear) ? 1.888f :
		(Racecar::Gear::Third == gear) ? 1.333f :
		(Racecar::Gear::Fourth == gear) ? 1.0f :
		(Racecar::Gear::Fifth == gear) ? 0.814f : 0.0f;
}

constexpr Racecar::Gear UpshiftGear(const Racecar::Gear& gear)
{
	return
		(Racecar::Gear::Reverse == gear) ? Racecar::Gear::Neutral :
		(Racecar::Gear::Neutral == gear) ? Racecar::Gear::First :
		(Racecar::Gear::First == gear) ? Racecar::Gear::Second :
		(Racecar::Gear::Second == gear) ? Racecar::Gear::Third :
		(Racecar::Gear::Third == gear) ? Racecar::Gear::Fourth : Racecar::Gear::Fifth;
}

constexpr Racecar::Gear DownshiftGear(const Racecar::Gear& gear)
{
	return
		(Racecar::Gear::Fifth == gear) ? Racecar::Gear::Fourth :
		(Racecar::Gear::Fourth == gear) ? Racecar::Gear::Third :
		(Racecar::Gear::Third == gear) ? Racecar::Gear::Second :
		(Racecar::Gear::Second == gear) ? Racecar::Gear::First :
		(Racecar::Gear::First == gear) ? Racecar::Gear::Neutral : Racecar::Gear::Reverse;
}

//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//

Racecar::Transmission::Transmission(void) :
	mInputShaftSpeed(0.0f),
	mOutputShaftSpeed(0.0f),
	mSelectedGear(Gear::Neutral),
	mHasClearedShift(true)
{
	SetInertia(Racecar::ComputeInertiaImperial(5.0f, 3.0f));
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::Transmission::~Transmission(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::Simulate(const RacecarControllerInterface& racecarController)
{
	SimulateShiftLogic(racecarController);

	RotatingBody& inputSource(GetExpectedInputSource());

	mInputShaftSpeed = inputSource.GetAngularVelocity();
	
	const float finalDriveRatio(4.30f); //Should be down the line, but meh.
	if (Gear::Neutral == mSelectedGear)
	{
		//Do nothing.
	}
	else
	{
		mOutputShaftSpeed = mInputShaftSpeed / RatioForGear(mSelectedGear) / finalDriveRatio;
		SetAngularVelocity(mOutputShaftSpeed);
	}
}

//--------------------------------------------------------------------------------------------------------------------//

//float Racecar::Transmission::ComputeDownstreamIntertia(const RotatingBody& fromSource) const
//{
//	if (Gear::Neutral == mSelectedGear)
//	{
//		return GetInertia();
//	}
//
//	return GetInertia() + GetExpectedOutputSource(0).ComputeDownstreamIntertia(fromSource) / 4.0f;
//}
//
////--------------------------------------------------------------------------------------------------------------------//
//
//float Racecar::Transmission::ComputeUpstreamInertia(const RotatingBody& fromSource) const
//{
//}
//
////--------------------------------------------------------------------------------------------------------------------//
//
//void Racecar::Transmission::OnApplyDownstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource)
//{
//}
//
////--------------------------------------------------------------------------------------------------------------------//
//
//void Racecar::Transmission::OnApplyUpstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource)
//{
//}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::Transmission::SimulateShiftLogic(const RacecarControllerInterface& racecarController)
{
	if (true == mHasClearedShift)
	{		
		if (true == racecarController.IsUpshift())
		{	//Upshift
			mSelectedGear = UpshiftGear(mSelectedGear);
			mHasClearedShift = false;
		}
		else if (true == racecarController.IsDownshift())
		{	//Downshift
			mSelectedGear = DownshiftGear(mSelectedGear);
			mHasClearedShift = false;
		}
	}
	else
	{
		if (false == racecarController.IsUpshift() && false == racecarController.IsDownshift())
		{
			mHasClearedShift = true;
		}
	}
}

//--------------------------------------------------------------------------------------------------------------------//

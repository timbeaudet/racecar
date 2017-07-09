///
/// @file
/// @details This is a simple racecar controller that provides all the controls a person would find inside a typical
///   racecar; throttle, brake, clutch pedals, steering wheel, shifter...
///
/// <!-- Copyright (c) 2017 Tim Beaudet - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_controller.h"

#include "../drive_train_simulation.h" //For kFixedTime
#include "../turtle_brains/system/unstable/tbu_input_device_manager.h"

//--------------------------------------------------------------------------------------------------------------------//

Racecar::RacecarControllerInterface::RacecarControllerInterface(void) :
	mThrottlePosition(0.0f),
	mBrakePosition(0.0f),
	mClutchPosition(0.0f),
	mSteeringPosition(0.0f),
	mIsUpshift(false),
	mIsDownshift(false)
{
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::RacecarControllerInterface::~RacecarControllerInterface(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::RacecarControllerInterface::UpdateControls(void)
{
	OnUpdateControls();
}

//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//

Racecar::DoNothingController::DoNothingController(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::DoNothingController::~DoNothingController(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::DoNothingController::OnUpdateControls(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

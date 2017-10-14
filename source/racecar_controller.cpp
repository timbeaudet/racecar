///
/// @file
/// @details This is a simple racecar controller that provides all the controls a person would find inside a typical
///   racecar; throttle, brake, clutch pedals, steering wheel, shifter...
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_controller.h"

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

Racecar::DoNothingController::DoNothingController(void) :
	RacecarControllerInterface()
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
//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//

Racecar::ProgrammaticController::ProgrammaticController(void) :
	RacecarControllerInterface()
{
}

//--------------------------------------------------------------------------------------------------------------------//


Racecar::ProgrammaticController::~ProgrammaticController(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::ProgrammaticController::OnUpdateControls(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

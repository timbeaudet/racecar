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
	mSteeringPosition(0.0f)
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
//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//

Racecar::PlayerRacecarController::PlayerRacecarController(void) :
	mThrottleAction(tbApplication::tbKeyUp),
	mBrakeAction(tbApplication::tbKeyDown),
	mClutchAction(tbApplication::tbKeyX),
	mTurnLeftAction(tbApplication::tbKeyLeft),
	mTurnRightAction(tbApplication::tbKeyRight)
{
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::PlayerRacecarController::~PlayerRacecarController(void)
{
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::PlayerRacecarController::OnUpdateControls(void)
{
	const float kKeyRate(4.0f * DriveTrainSimulation::kFixedTime); //half a second, to fully pressed/released.
	SetThrottlePosition(GetThrottlePosition() + ((mThrottleAction.IsDown()) ? kKeyRate : -kKeyRate));
	SetBrakePosition(GetBrakePosition() + ((mBrakeAction.IsDown()) ? kKeyRate : -kKeyRate));
	SetClutchPosition(GetClutchPosition() + ((mClutchAction.IsDown()) ? kKeyRate : -kKeyRate));

	float steeringPosition(GetSteeringPosition());
	if (mTurnLeftAction.IsDown())
	{
		steeringPosition -= kKeyRate;
	}
	else if (mTurnRightAction.IsDown())
	{
		steeringPosition += kKeyRate;
	}
	else
	{	//Recenter the steering wheel position when not turning left or right.
		if (steeringPosition > kKeyRate) { steeringPosition -= kKeyRate; }
		else if (steeringPosition < -kKeyRate) { steeringPosition += kKeyRate; }
		else { steeringPosition = 0.0f; }
	}

	SetSteeringPosition(steeringPosition);
}

//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------//

Racecar::PlayerWheelRacecarController::PlayerWheelRacecarController(void)
{
	tbSystem::Unstable::theInputDeviceManager.ConnectWithDevices();
}

//--------------------------------------------------------------------------------------------------------------------//

Racecar::PlayerWheelRacecarController::~PlayerWheelRacecarController(void)
{
	tbSystem::Unstable::theInputDeviceManager.DisconnectFromDevices();
}

//--------------------------------------------------------------------------------------------------------------------//

void Racecar::PlayerWheelRacecarController::OnUpdateControls(void)
{
	tbSystem::Unstable::theInputDeviceManager.PollInputDevices();

	SetSteeringPosition(tbSystem::Unstable::theInputDeviceManager.GetAxisPercentage(0, 0, tbSystem::Unstable::InputDeviceManager::PercentageRange::kNegativeToPositiveOne));
	SetThrottlePosition(1.0f - tbSystem::Unstable::theInputDeviceManager.GetAxisPercentage(0, 1, tbSystem::Unstable::InputDeviceManager::PercentageRange::kZeroToPositiveOne));
	SetBrakePosition(1.0f - tbSystem::Unstable::theInputDeviceManager.GetAxisPercentage(0, 5, tbSystem::Unstable::InputDeviceManager::PercentageRange::kZeroToPositiveOne));
	SetClutchPosition(1.0f - tbSystem::Unstable::theInputDeviceManager.GetAxisPercentage(0, 6, tbSystem::Unstable::InputDeviceManager::PercentageRange::kZeroToPositiveOne));
}

//--------------------------------------------------------------------------------------------------------------------//

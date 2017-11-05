///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_body.h"
#include "racecar_wheel.h"

//-------------------------------------------------------------------------------------------------------------------//

Racecar::RacecarBody::RacecarBody(const Real& mass) :
	mWheels{nullptr, nullptr, nullptr, nullptr},
	mMass(mass),
	mLinearVelocity(0.0)
{
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::RacecarBody::~RacecarBody(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RacecarBody::ControllerChange(const Racecar::RacecarControllerInterface& racecarController)
{
	((void)racecarController);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RacecarBody::Simulate(const Real fixedTime)
{
	((void)fixedTime);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RacecarBody::SetLinearVelocity(const Real& linearVelocity)
{
	mLinearVelocity = linearVelocity;
	for (Wheel* wheel : mWheels)
	{
		if (nullptr == wheel)
		{
			continue;
		}
		
		wheel->SetLinearVelocity(linearVelocity);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::RacecarBody::GetTotalMass(void) const
{
	Real totalMass(mMass);
	for (Wheel* wheel : mWheels)
	{
		if (nullptr != wheel)
		{
			totalMass += wheel->GetMass();
		}
	}
	return totalMass;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RacecarBody::ApplyLinearImpulse(const Real& linearImpulse)
{
	const Real totalMass(GetTotalMass());
	error_if(totalMass < 0.001, "Total Mass is too small.");
	SetLinearVelocity(mLinearVelocity + linearImpulse / totalMass);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RacecarBody::OnLinearVelocityChange(const Real& changeInLinearVelocity)
{
	SetLinearVelocity(mLinearVelocity + changeInLinearVelocity);
}

//-------------------------------------------------------------------------------------------------------------------//

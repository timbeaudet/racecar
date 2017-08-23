///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_body.h"
#include "racecar_wheel.h"

//-------------------------------------------------------------------------------------------------------------------//

Racecar::RacecarBody::RacecarBody(const Real& mass) :
	mWheels{nullptr, nullptr, nullptr, nullptr},
	mMass(mass),
	mLinearAcceleration(0.0),
	mLinearVelocity(0.0)
{
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::RacecarBody::~RacecarBody(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RacecarBody::Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real fixedTime)
{
	((void)racecarController);

	SetLinearVelocity(mLinearVelocity + mLinearAcceleration * fixedTime);
	mLinearAcceleration = 0.0;
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

void Racecar::RacecarBody::ApplyForce(const Real& forceInNewtons)
{
	const Real totalMass(GetTotalMass());
	mLinearAcceleration += forceInNewtons / totalMass;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RacecarBody::OnApplyLinearAcceleration(const Real& changeInAcceleration)
{
	mLinearAcceleration += changeInAcceleration;
}

//-------------------------------------------------------------------------------------------------------------------//

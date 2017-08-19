///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_body.h"

//-------------------------------------------------------------------------------------------------------------------//

Racecar::RacecarBody::RacecarBody(const Real& mass) :
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

	mLinearVelocity += mLinearAcceleration * fixedTime;
	mLinearAcceleration = 0.0;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RacecarBody::ApplyForce(const Real& forceInNewtons)
{
	mLinearAcceleration += forceInNewtons / mMass;
}

//-------------------------------------------------------------------------------------------------------------------//

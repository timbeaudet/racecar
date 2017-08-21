///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_wheel.h"
#include "racecar_body.h"
#include "racecar_controller.h"

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Wheel::Wheel(const Real& massInKilograms, const Real& radiusInMeters) :
	RotatingBody(massInKilograms * (radiusInMeters * radiusInMeters)), //kg-m^2
	mMass(massInKilograms),
	mRadius(radiusInMeters),
	mLinearAcceleration(0.0),
	mLinearVelocity(0.0),
	mRacecarBody(nullptr),
	mIsOnGround(false)
{
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Wheel::~Wheel(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::SetRacecarBody(RacecarBody* racecarBody)
{
	error_if(mRacecarBody != nullptr, "This wheel already has a racecar body.");
	mRacecarBody = racecarBody;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real& fixedTime)
{
	const RotatingBody* inputSource(GetInputSource());
	if (nullptr != inputSource)
	{
		SetAngularVelocity(inputSource->GetAngularVelocity());
	}

	if (racecarController.GetBrakePosition() > Racecar::PercentTo<float>(1.0f))
	{
		//The brake can apply negative force - need to clamp it
		//HELL - Need to do it correctly!!
		ApplyUpstreamTorque(-GetAngularVelocity() * (0.83f * racecarController.GetBrakePosition()) * fixedTime, *this);
	}

	///
	if (true == IsOnGround())
	{
		const Real expectedAngularVelocity(mLinearVelocity / mRadius); //radians / sec
		const Real difference = GetAngularVelocity() - expectedAngularVelocity; //faster positive, slower negative

		const Real totalMass(mMass + ((nullptr == mRacecarBody) ? 0.0f : mRacecarBody->GetMass()));
		const Real totalInertia(ComputeUpstreamInertia(*this));
		const Real impulse = totalMass * totalInertia * (GetLinearVelocity() - GetAngularVelocity() * mRadius) / (totalMass * (mRadius * mRadius) + totalInertia);
		ApplyUpstreamTorque(impulse * mRadius / fixedTime, *this);
		if (nullptr != mRacecarBody)
		{ 
			mRacecarBody->ApplyForce(impulse * -2.0 / fixedTime);
		}
		else
		{
			mLinearAcceleration += -2.0 * impulse / totalInertia / fixedTime;
		}
		//That should have applied correct force to racecar body, should it exist.
	}

	RotatingBody::Simulate();

	mLinearVelocity += mLinearAcceleration * fixedTime;
	mLinearAcceleration = 0.0;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::AddAngularAcceleration(const Real& angularAcceleration)
{
	RotatingBody::AddAngularAcceleration(angularAcceleration);

	if (true == mIsOnGround)
	{
		ApplyForceToGroundFrom(angularAcceleration);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::OnApplyDownstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource)
{
	RotatingBody::OnApplyDownstreamAcceleration(changeInAcceleration, fromSource);

	if (true == mIsOnGround)
	{
		ApplyForceToGroundFrom(changeInAcceleration);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::OnApplyUpstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource)
{
	RotatingBody::OnApplyUpstreamAcceleration(changeInAcceleration, fromSource);

	if (true == mIsOnGround)
	{
		ApplyForceToGroundFrom(changeInAcceleration);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::ApplyForceToGroundFrom(const Real& angularAcceleration)
{
	const Real appliedTorque(angularAcceleration * GetInertia());
	const Real appliedForce(appliedTorque / mRadius);
	
	if (nullptr != mRacecarBody)
	{
		const Real changeInAcceleration(appliedForce / (mMass + mRacecarBody->GetMass()));
		mLinearAcceleration += changeInAcceleration;
		mRacecarBody->OnApplyLinearAcceleration(changeInAcceleration);
	}
	else
	{
		mLinearAcceleration += appliedForce / mMass;
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::SetOnGround(bool isOnGround)
{
	mIsOnGround = isOnGround;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Wheel::GetWheelSpeedMPH(void) const
{
	const Real speedFeetPerSecond(GetAngularVelocity() * (22.0 / 2.0) / 12.0);
	const Real speedMPH(speedFeetPerSecond * 60 * 60 / 5280.0);
	return fabs(speedMPH);
}

//-------------------------------------------------------------------------------------------------------------------//

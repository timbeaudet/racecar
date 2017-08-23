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

	///This is currently assuming an INFINITE amount of friction which will cause the tire never to lock up, and always
	///match the speed of the racecar, but will slow the car/speed the wheel or speed the car/slow the wheel as necessary
	///when making contact with the ground that has infinite friction!
	ApplyGroundFriction(fixedTime);

	RotatingBody::Simulate();

	mLinearVelocity += mLinearAcceleration * fixedTime;
	mLinearAcceleration = 0.0;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Wheel::ComputeDownstreamInertia(const RotatingBody& fromSource) const
{
	if (true == mIsOnGround && nullptr != mRacecarBody)
	{
		return RotatingBody::ComputeDownstreamInertia(fromSource) + (mRacecarBody->GetMass() * mRadius * mRadius);
	}

	return RotatingBody::ComputeDownstreamInertia(fromSource);
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Wheel::ComputeUpstreamInertia(const RotatingBody& fromSource) const
{
	if (true == mIsOnGround && nullptr != mRacecarBody)
	{
		return RotatingBody::ComputeUpstreamInertia(fromSource) + (mRacecarBody->GetMass() * mRadius * mRadius);
	}

	return RotatingBody::ComputeUpstreamInertia(fromSource);
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
	const Real totalInertia((false == IsOnGround() || nullptr == mRacecarBody) ? GetInertia() : ((mRacecarBody->GetMass() * 0.25 * 0.25) + GetInertia()));
	const Real appliedTorque(angularAcceleration * totalInertia);
	const Real appliedForce(appliedTorque / mRadius);
	
	if (nullptr != mRacecarBody)
	{
		const Real changeInAcceleration(appliedForce / (mRacecarBody->GetTotalMass()));
		mLinearAcceleration += changeInAcceleration;
		mRacecarBody->OnApplyLinearAcceleration(changeInAcceleration);
	}
	else
	{
		mLinearAcceleration += appliedForce / mMass;
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::ApplyGroundFriction(const Real& fixedTime)
{
	if (true == IsOnGround())
	{
		const Real expectedAngularVelocity(mLinearVelocity / mRadius); //radians / sec
		const Real difference = GetAngularVelocity() - expectedAngularVelocity; //faster positive, slower negative

		const Real totalMass(mMass + ((nullptr == mRacecarBody) ? 0.0f : mRacecarBody->GetMass()));
		//TODO: Understand: Calling RotatingBody::Compute to avoid adding the mass/inertia of the car a second time which
		//may have been inflating the size of the impulse to be applied for 'infinite' friction.

		//TODO: Understand: We are pretending the car is not contacting the ground in order to apply the correct
		//rotational acceleration / torques and then separately the linear, which we use to need to negate.
		mIsOnGround = false;
		const Real totalInertia(ComputeUpstreamInertia(*this));
		const Real impulse = totalMass * totalInertia * (GetLinearVelocity() - GetAngularVelocity() * mRadius) / (totalMass * (mRadius * mRadius) + totalInertia);
		ApplyUpstreamTorque(impulse * mRadius / fixedTime, *this);
		mIsOnGround = true;

		//TODO: Understand:
		//OLD: We use to apply -2.0 * impulse below, and this is the note of why:
		//The following is for the 'reversal' of the force/torque applied by the impulse. The drive-train currently
		//may not have positive/negative values correct, or something? So we do 2 times the impulse to first negate
		//the impulse then make it in the direction expected by the unit tests... Obviously this needs investigating.
		//
		//NEW: We now apply -1.0 * impulse below for linear accelerations since they are NOT being applied during the
		//ApplyUpstreamTorque since we are using the isOnGround = false hack that is documented above!
		if (nullptr != mRacecarBody)
		{
			mRacecarBody->ApplyForce(impulse * -1.0 / fixedTime);
		}
		else
		{
			mLinearAcceleration += -1.0 * impulse / totalInertia / fixedTime;
		}
		//That should have applied correct force to racecar body, should it exist.
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

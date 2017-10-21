///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_wheel.h"
#include "racecar_body.h"
#include "racecar_controller.h"

const Racecar::Real Racecar::Wheel::kInfiniteFriction(-1.0);

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Wheel::Wheel(const Real& massInKilograms, const Real& radiusInMeters) :
	RotatingBody(massInKilograms * (radiusInMeters * radiusInMeters)), //kg-m^2
	mMass(massInKilograms),
	mRadius(radiusInMeters),
	mLinearAcceleration(0.0),
	mLinearVelocity(0.0),
	mGroundFrictionCoefficient(-1.0),
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
	((void)racecarController);

	//if (racecarController.GetBrakePosition() > Racecar::PercentTo<float>(1.0f))
	//{
	//	//The brake can apply negative force - need to clamp it
	//	//HELL - Need to do it correctly!!
	//	ApplyUpstreamTorque(-GetAngularVelocity() * ComputeUpstreamInertia(*this) * (0.83f * racecarController.GetBrakePosition()), *this);
		
		//Just change to Upstream and/or carbody mass included, and this should be good for braking.
		//const Real totalInertia(ComputeDownstreamInertia(*this));
		//const Real maximumImpulse(totalInertia * GetAngularVelocity()); //kg*m^2 / s
		//const Real actualImpulse(mResistanceTorque * fixedTime); //kg*m^2 / s
		//const Real appliedImpulse((actualImpulse > maximumImpulse) ? maximumImpulse : actualImpulse);
		////The /fixedTime is to apply this as an impulse, deeper down it will *fixedTime.
		//ApplyDownstreamTorque(appliedImpulse / fixedTime * -Racecar::Sign(GetAngularVelocity()), *this);
	//}

	///This is currently assuming an INFINITE amount of friction which will cause the tire never to lock up, and always
	///match the speed of the racecar, but will slow the car/speed the wheel or speed the car/slow the wheel as necessary
	///when making contact with the ground that has infinite friction!
	ApplyGroundFriction(fixedTime);

	RotatingBody::Simulate();

	mLinearVelocity += mLinearAcceleration * fixedTime;
	mLinearAcceleration = 0.0;

	const RotatingBody* inputSource(GetInputSource());
	if (nullptr != inputSource)
	{
	//	error_if(fabs(GetAngularVelocity() - inputSource->GetAngularVelocity()) > Racecar::kElipson,
	//		"The wheel speed does not match the input velocity after Simulate().");
		SetAngularVelocity(inputSource->GetAngularVelocity());
	}
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
		const Real velocityDifference(GetAngularVelocity() * mRadius - GetLinearVelocity());
		const Real impulse = (velocityDifference * totalInertia * totalMass) / (totalInertia + ((mRadius * mRadius) * totalMass));

		const Real frictionImpulse(ComputeFrictionForce(totalMass) * Racecar::Sign(velocityDifference) * fixedTime);
		const Real appliedImpulse((fabs(impulse) <= fabs(frictionImpulse) || mGroundFrictionCoefficient <= 0.0) ?
			impulse : frictionImpulse);
		
		ApplyUpstreamTorque(-appliedImpulse / fixedTime * mRadius, *this);
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
			mRacecarBody->ApplyForce(appliedImpulse / fixedTime);
		}
		else
		{
			mLinearAcceleration += appliedImpulse / fixedTime / totalMass;
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::SetOnGround(bool isOnGround, const Real& frictionCoefficient)
{
	mIsOnGround = isOnGround;
	mGroundFrictionCoefficient = frictionCoefficient;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Wheel::GetWheelSpeedMPH(void) const
{
	const Real speedFeetPerSecond(GetAngularVelocity() * (22.0 / 2.0) / 12.0);
	const Real speedMPH(speedFeetPerSecond * 60 * 60 / 5280.0);
	return fabs(speedMPH);
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Wheel::ComputeFrictionForce(const Real& totalMass)
{
	const Real normalForce(Racecar::GetGravityConstant() * totalMass);
	return normalForce * mGroundFrictionCoefficient;
}

//-------------------------------------------------------------------------------------------------------------------//

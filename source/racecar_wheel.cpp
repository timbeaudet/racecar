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
	mLinearVelocity(0.0),
	mGroundFrictionCoefficient(-1.0),
	mMaximumBrakingTorque(100.0), //Nm
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
	///Compute the impulse it would take to stop the wheel / car + connections, compare that against the
	///actual impulse that would be applied based on braking torque, use the lower of those values in the
	///correct direction, and apply the upstream torque to slow the wheel + connections.
	const Real totalInertia(ComputeUpstreamInertia());
	const Real maximumImpulse(totalInertia * fabs(GetAngularVelocity())); //kg*m^2 / s
	const Real actualImpulse(mMaximumBrakingTorque * racecarController.GetBrakePosition() * fixedTime); //kg*m^2 / s
	const Real appliedImpulse((actualImpulse > maximumImpulse) ? maximumImpulse : actualImpulse);
	if (appliedImpulse > kElipson)
	{
		ApplyUpstreamAngularImpulse(appliedImpulse * -Racecar::Sign(GetAngularVelocity()));
	}

	///This is currently assuming an INFINITE amount of friction which will cause the tire never to lock up, and always
	///match the speed of the racecar, but will slow the car/speed the wheel or speed the car/slow the wheel as necessary
	///when making contact with the ground that has infinite friction!
	ApplyGroundFriction(fixedTime);

	RotatingBody::Simulate();

	//const RotatingBody* inputSource(GetInputSource());
	//if (nullptr != inputSource)
	//{
	////	error_if(fabs(GetAngularVelocity() - inputSource->GetAngularVelocity()) > Racecar::kElipson,
	////		"The wheel speed does not match the input velocity after Simulate().");
	//	SetAngularVelocity(inputSource->GetAngularVelocity());
	//}
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Wheel::ComputeDownstreamInertia(void) const
{
	if (true == mIsOnGround && nullptr != mRacecarBody)
	{
		const Real carInertia((mRacecarBody->GetMass() * mRadius * mRadius));
		return RotatingBody::ComputeDownstreamInertia() + carInertia;
	}

	return RotatingBody::ComputeDownstreamInertia();
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Wheel::ComputeUpstreamInertia(void) const
{
	if (true == mIsOnGround && nullptr != mRacecarBody)
	{
		const Real carInertia((mRacecarBody->GetMass() * mRadius * mRadius));
		return RotatingBody::ComputeUpstreamInertia() + carInertia;
	}

	return RotatingBody::ComputeUpstreamInertia();
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity)
{
	RotatingBody::OnDownstreamAngularVelocityChange(changeInAngularVelocity);

	if (true == mIsOnGround)
	{
		//TODO: DriveTrain: This is NOT correct. It use to take changeInAcceleratoin. So:
		//  angAcceleration += changeInAcceleration;
		//  angVelocity += angAcceleration * deltaTime;
		//
		// We don't have a delta / fixed time here. And changeInAngularVelocity is:
		//  angVelocity += changeInAngularVelocity.
//		ApplyForceToGroundFrom(changeInAngularVelocity);

		const Real changeInLinearVelocity = changeInAngularVelocity * mRadius;
		if (nullptr != mRacecarBody)
		{
			mRacecarBody->OnLinearVelocityChange(changeInLinearVelocity);
		}
		else
		{
			mLinearVelocity += changeInLinearVelocity;
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Wheel::OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity)
{
	RotatingBody::OnUpstreamAngularVelocityChange(changeInAngularVelocity);

	if (true == mIsOnGround)
	{
		//TODO: DriveTrain: This is NOT correct. It use to take changeInAcceleratoin. So:
		//  angAcceleration += changeInAcceleration;
		//  angVelocity += angAcceleration * deltaTime;
		//
		// We don't have a delta / fixed time here. And changeInAngularVelocity is:
		//  angVelocity += changeInAngularVelocity.
//		ApplyForceToGroundFrom(changeInAngularVelocity);

		const Real changeInLinearVelocity = changeInAngularVelocity * mRadius;
		if (nullptr != mRacecarBody)
		{
			mRacecarBody->OnLinearVelocityChange(changeInLinearVelocity);
		}
		else
		{
			mLinearVelocity += changeInLinearVelocity;
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------//

//void Racecar::Wheel::ApplyForceToGroundFrom(const Real& changeInAngularVelocity)
//{
//	//TODO: DriveTrain: This is NOT correct. It use to take changeInAcceleratoin. So:
//	//  angAcceleration += changeInAcceleration;
//	//  angVelocity += angAcceleration * deltaTime;
//	//
//	// We don't have a delta / fixed time here. And changeInAngularVelocity is:
//	//  angVelocity += changeInAngularVelocity.
//	//
//	//See OnUpstreamAngularVelocityChange() and OnDownstreamAngularVelocityChange().
//	const Real totalInertia((false == IsOnGround() || nullptr == mRacecarBody) ? GetInertia() : ((mRacecarBody->GetMass() * 0.25 * 0.25) + GetInertia()));
//	const Real appliedTorque(angularAcceleration * totalInertia);
//	const Real appliedForce(appliedTorque / mRadius);
//	
//	if (nullptr != mRacecarBody)
//	{
//		const Real totalMass(mRacecarBody->GetTotalMass()); //This automatically includes mass of the wheel.
//
//		const Real changeInAcceleration(appliedForce / totalMass);
//		mLinearAcceleration += changeInAcceleration;
//		
//		//mLinearVelocity += mLinearAcceleration * fixedTime;
//
//		mRacecarBody->OnApplyLinearAcceleration(changeInAcceleration);
//	}
//	else
//	{
//		mLinearAcceleration += appliedForce / mMass;
//	}
//}

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
		const Real totalInertia(ComputeUpstreamInertia());
		const Real velocityDifference(GetAngularVelocity() * mRadius - GetLinearVelocity());
		const Real impulse = (velocityDifference * totalInertia * totalMass) / (totalInertia + ((mRadius * mRadius) * totalMass));

		const Real frictionImpulse(ComputeFrictionForce(totalMass) * Racecar::Sign(velocityDifference) * fixedTime);
		const Real appliedImpulse((fabs(impulse) <= fabs(frictionImpulse) || mGroundFrictionCoefficient <= 0.0) ?
			impulse : frictionImpulse);
		
		if (fabs(appliedImpulse) > kElipson)
		{	//Ensure there is some amount of frictional impulse, to avoid NaN.
			ApplyUpstreamAngularImpulse(-appliedImpulse * mRadius);

			//TODO: Understand:
			//OLD: We use to apply -2.0 * impulse below, and this is the note of why:
			//The following is for the 'reversal' of the force/torque applied by the impulse. The drive-train currently
			//may not have positive/negative values correct, or something? So we do 2 times the impulse to first negate
			//the impulse then make it in the direction expected by the unit tests... Obviously this needs investigating.
			//
			//NEW: We now apply -1.0 * impulse below for linear accelerations since they are NOT being applied during the
			//ApplyUpstreamTorque since we are using the isOnGround = false hack that is documented above!
			if (nullptr != mRacecarBody)
			{	//TODO: DriveTrain: Change ApplyForce to ApplyLinearImpulse for consistency reasons.
				mRacecarBody->ApplyLinearImpulse(appliedImpulse);
			}
			else
			{
				//mLinearAcceleration += appliedImpulse / fixedTime / totalMass;
			}
		}
		mIsOnGround = true;
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

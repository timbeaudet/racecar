///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Wheel_h_
#define _Racecar_Wheel_h_

#include "rotating_body.h"

namespace Racecar
{
	class RacecarControllerInterface;
	class RacecarBody;

	class Wheel : public RotatingBody
	{
	public:
		static const Real kInfiniteFriction;

		explicit Wheel(const Real& massInKilograms, const Real& radiusInMeters); //kg-m^2
		virtual ~Wheel(void);

		///
		/// @details Computes the speed of the wheel in miles per hour, from the rate at which it is spinning at.
		///
		/// @note This is NOT connected to linear velocity, it is connected with angular velocity. If the car was 
		///   sitting still on ice, and the driver hit the throttle, the wheels could spin while remaining stationary.
		///
		Real GetWheelSpeedMPH(void) const;

		///
		/// @details Returns the radius of the wheel in meters.
		///
		inline Real GetRadius(void) const { return mRadius; }

		inline bool IsOnGround(void) const { return mIsOnGround; }
		void SetOnGround(bool isOnGround, const Real& frictionCoefficient);

		inline const Real& GetLinearVelocity(void) const { return mLinearVelocity; }
		inline void SetLinearVelocity(const Real& linearVelocity) { mLinearVelocity = linearVelocity; }

		const Real& GetMass(void) const { return mMass; }
		void SetRacecarBody(RacecarBody* racecarBody);

		inline void SetMaximumBrakingTorque(const Real& maximumBrakingTorque) { mMaximumBrakingTorque = maximumBrakingTorque; }

		virtual Real ComputeDownstreamInertia(void) const;
		virtual Real ComputeUpstreamInertia(void) const;

	protected:
		virtual void OnControllerChange(const RacecarControllerInterface& racecarController) override;
		virtual void OnSimulate(const Real& fixedTime) override;

		virtual void OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity) override;
		virtual void OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity) override;

		//void ApplyForceToGroundFrom(const Real& angularAcceleration);
		void ApplyGroundFriction(const Real& fixedTime);

	private:
		Real ComputeFrictionForce(const Real& totalMass);

		Real mMass;
		Real mRadius;
		Real mLinearVelocity;
		Real mGroundFrictionCoefficient; //If <= 0.0 assume infinite friction!
		Real mMaximumBrakingTorque;
		Real mBrakePedalPosition;
		RacecarBody* mRacecarBody;
		bool mIsOnGround;
	};

};	/* namespace Racecar */

#endif /* _Racecar_Wheel_h_ */

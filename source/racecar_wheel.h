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
		///
		///
		void Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real& fixedTime = Racecar::kFixedTimeStep);

		///
		///
		///
		Real GetWheelSpeedMPH(void) const;

		inline bool IsOnGround(void) const { return mIsOnGround; }
		void SetOnGround(bool isOnGround, const Real& frictionCoefficient);

		inline const Real& GetLinearVelocity(void) const { return mLinearVelocity; }
		inline void SetLinearVelocity(const Real& linearVelocity) { mLinearVelocity = linearVelocity; }

		const Real& GetMass(void) const { return mMass; }
		void SetRacecarBody(RacecarBody* racecarBody);

	protected:
		virtual Real ComputeDownstreamInertia(const RotatingBody& fromSource) const;
		virtual Real ComputeUpstreamInertia(const RotatingBody& fromSource) const;
		virtual void AddAngularAcceleration(const Real& angularAcceleration) override;
		virtual void OnApplyDownstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;
		virtual void OnApplyUpstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;

		void ApplyForceToGroundFrom(const Real& angularAcceleration);
		void ApplyGroundFriction(const Real& fixedTime);

	private:
		Real ComputeFrictionForce(const Real& totalMass);

		Real mMass;
		Real mRadius;
		Real mLinearAcceleration;
		Real mLinearVelocity;
		Real mGroundFrictionCoefficient; //If <= 0.0 assume infinite friction!
		RacecarBody* mRacecarBody;
		bool mIsOnGround;
		
	};

};	/* namespace Racecar */

#endif /* _Racecar_Wheel_h_ */

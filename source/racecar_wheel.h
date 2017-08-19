///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Wheel_h_
#define _Racecar_Wheel_h_

#include "rotating_body.h"

namespace Racecar
{
	class RacecarControllerInterface;

	class Wheel : public RotatingBody
	{
	public:
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
		inline void SetOnGround(bool isOnGround) { mIsOnGround = isOnGround; }

		inline const Real& GetLinearVelocity(void) const { return mLinearVelocity; }

	protected:
		virtual void AddAngularAcceleration(const Real& angularAcceleration) override;
		virtual void OnApplyDownstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;
		virtual void OnApplyUpstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;

		void ApplyForceToGroundFrom(const Real& angularAcceleration);

	private:
		Real mMass;
		Real mRadius;
		Real mLinearAcceleration;
		Real mLinearVelocity;
		bool mIsOnGround;
	};

};	/* namespace Racecar */

#endif /* _Racecar_Wheel_h_ */

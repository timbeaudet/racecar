///
/// @file
/// @details This is a basic simulation for a locked differential, where both wheels spin same speed.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_LockedDifferential_h_
#define _Racecar_LockedDifferential_h_

#include "rotating_body.h"
#include "racecar_transmission.h" //For GearJoint

namespace Racecar
{

	class RacecarControllerInterface;

	class LockedDifferential : public RotatingBody
	{
	public:
		explicit LockedDifferential(const Real& momentOfInertia, const Real& finalDriveRatio);
		virtual ~LockedDifferential(void);

		void Simulate(RacecarControllerInterface& racecarController, const Real& fixedTime = Racecar::kFixedTimeStep);

		virtual Real ComputeDownstreamInertia(const RotatingBody& fromSource) const override;
		virtual Real ComputeUpstreamInertia(const RotatingBody& fromSource) const override;

	protected:
		virtual void OnApplyDownstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;
		virtual void OnApplyUpstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;

	private:
		GearJoint mFinalDriveJoint;
	};

};	/* namespace Racecar */

#endif /* _Racecar_LockedDifferential_h_ */

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

		virtual Real ComputeDownstreamInertia(void) const override;
		virtual Real ComputeUpstreamInertia(void) const override;

	protected:
		virtual void OnSimulate(const Real& fixedTime) override;
		virtual void OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity) override;
		virtual void OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity) override;

	private:
		GearJoint mFinalDriveJoint;
	};

};	/* namespace Racecar */

#endif /* _Racecar_LockedDifferential_h_ */

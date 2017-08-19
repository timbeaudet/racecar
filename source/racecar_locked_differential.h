///
/// @file
/// @details This is a basic simulation for a locked differential, where both wheels spin same speed.
///
/// <!-- Copyright (c) 2017 Tim Beaudet - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_LockedDifferential_h_
#define _Racecar_LockedDifferential_h_

#include "rotating_body.h"

namespace Racecar
{

	class RacecarControllerInterface;

	class LockedDifferential : public RotatingBody
	{
	public:
		explicit LockedDifferential(const Real& finalDriveRatio, const Real& momentOfInertia);
		virtual ~LockedDifferential(void);

		void Simulate(RacecarControllerInterface& racecarController, const Real& fixedTime = Racecar::kFixedTimeStep);
	
	protected:
		virtual Real ComputeDownstreamInertia(const RotatingBody& fromSource) const override;
		virtual Real ComputeUpstreamInertia(const RotatingBody& fromSource) const override;
		virtual void OnApplyDownstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;
		virtual void OnApplyUpstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;

	private:
		const Real mFinalDriveRatio;
	};

};	/* namespace Racecar */

#endif /* _Racecar_LockedDifferential_h_ */

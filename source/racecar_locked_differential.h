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
		explicit LockedDifferential(const float finalDriveRatio);
		virtual ~LockedDifferential(void);

		void Simulate(void);
	
	protected:
		//virtual float ComputeDownstreamIntertia(const RotatingBody& fromSource) const override;
		//virtual float ComputeUpstreamInertia(const RotatingBody& fromSource) const override;
		//virtual void OnApplyDownstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource) override;
		//virtual void OnApplyUpstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource) override;

	private:
		const float mFinalDriveRatio;
	};

};	/* namespace Racecar */

#endif /* _Racecar_LockedDifferential_h_ */

///
/// @file
/// @details This is a simple simulation for a clutch, that applies forces to two separate rotating bodies via
///   a frictional clutch disk.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Clutch_h_
#define _Racecar_Clutch_h_

#include "rotating_body.h"

namespace Racecar
{
	class RacecarControllerInterface;

	class Clutch : public RotatingBody
	{
	public:
		explicit Clutch(const Real& momentOfInertia);
		virtual ~Clutch(void);

		///
		///
		///
		void Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real& fixedTime = Racecar::kFixedTimeStep);

		///
		///
		///
		Real GetClutchEngagement(void) const { return mClutchEngagement; }

	protected:
		virtual Real ComputeDownstreamInertia(const RotatingBody& fromSource) const override;
		virtual void OnApplyDownstreamAcceleration(const Real changeInAcceleration, const RotatingBody& fromSource);

	private:
		static Real ClutchPedalToClutchForce(const float pedalInput);
		Real ComputeFrictionalTorque(void) const;

		Real mClutchEngagement; //0.0f for disengaged, 1.0f for completely engaged.
	};
};	/* namespace Racecar */

#endif /* _Racecar_Clutch_h_ */

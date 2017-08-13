///
/// @file
/// @details This is a simple simulation for a clutch, that applies forces to two separate rotating bodies via
///   a frictional clutch disk.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
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
		Clutch(const float momentOfInertia);
		virtual ~Clutch(void);

		///
		///
		///
		void Simulate(const Racecar::RacecarControllerInterface& racecarController);

		///
		///
		///
		float GetClutchEngagement(void) const { return mClutchEngagement; }

	protected:
		virtual float ComputeDownstreamInertia(const RotatingBody& fromSource) const override;
		virtual void OnApplyDownstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource);

	private:
		static float ClutchPedalToClutchForce(const float pedalInput);
		float ComputeFrictionalTorque(void) const;

		float mClutchEngagement; //0.0f for disengaged, 1.0f for completely engaged.
	};
};	/* namespace Racecar */

#endif /* _Racecar_Clutch_h_ */

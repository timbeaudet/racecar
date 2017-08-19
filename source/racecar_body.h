///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Body_h_
#define _Racecar_Body_h_

#include "racecar.h"

namespace Racecar
{
	class RacecarControllerInterface;

	class RacecarBody
	{
	public:
		explicit RacecarBody(const Real& mass); //kg
		virtual ~RacecarBody(void);

		///
		///
		///
		void Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real fixedTime = Racecar::kFixedTimeStep);

		///
		/// @details Applies a force to the body of the car, which will effect the accelerations for the next Simulate step.
		///
		void ApplyForce(const Real& forceInNewtons);

	protected:

	private:
		Real mMass;
		Real mLinearAcceleration;
		Real mLinearVelocity;
	};
};	/* namespace Racecar */

#endif /* _Racecar_Body_h_ */

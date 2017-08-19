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
		explicit Wheel(const Real momentOfInertia); //kg-m^2
		virtual ~Wheel(void);

		///
		///
		///
		void Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real fixedTime = Racecar::kFixedTimeStep);

		///
		///
		///
		Real GetWheelSpeedMPH(void) const;

	protected:

	private:
	};
};	/* namespace Racecar */

#endif /* _Racecar_Wheel_h_ */

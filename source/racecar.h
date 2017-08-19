///
/// @file
/// @details Provides includes for the different components of the racecar drivetrain.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Racecar_h_
#define _Racecar_Racecar_h_

#include <stdexcept>

namespace Racecar
{
	typedef double Real;

	static const Real kFixedTimeStep(0.01);

	inline constexpr Real ComputeInertiaMetric(const Real& massInKilograms, const Real& radiusInMeters)
	{
		return massInKilograms * (radiusInMeters * radiusInMeters);
	}

#define error_if(test, message, ...)  if(test) { printf(message, ##__VA_ARGS__); throw std::runtime_error(message); }

} /* namespace Racecar */

#endif /* _Racecar_Racecar_h_ */

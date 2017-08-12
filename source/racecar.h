///
/// @file
/// @details Provides includes for the different components of the racecar drivetrain.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Racecar_h_
#define _Racecar_Racecar_h_

namespace Racecar
{

	inline constexpr float ComputeInertiaMetric(float massInKilograms, float radiusInMeters)
	{
		return massInKilograms * (radiusInMeters * radiusInMeters);
	}

} /* namespace Racecar */

#endif /* _Racecar_Racecar_h_ */

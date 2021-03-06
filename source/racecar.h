///
/// @file
/// @details Provides includes for the different components of the racecar drivetrain.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Racecar_h_
#define _Racecar_Racecar_h_

#include <cmath>
#include <stdexcept>

namespace Racecar
{
	typedef double Real;

	static const Real kFixedTimeStep(0.01);
	static const Real kEpsilon(0.00001);
	extern Real kGravityConstant;

	inline constexpr Real ComputeInertiaMetric(const Real& massInKilograms, const Real& radiusInMeters)
	{
		return massInKilograms * (radiusInMeters * radiusInMeters);
	}

	inline Real GetGravityConstant(void) { return kGravityConstant; }
	inline void SetGravityConstant(const Real& gravity) { kGravityConstant = gravity; }

	template <typename Type> int Sign(const Type& value)
	{
		return (Type(0) < value) - (value < Type(0));
	}

#ifndef warning_if
#define warning_if(test, message, ...)  if(test) { printf(message, ##__VA_ARGS__); }
#endif

#ifndef error_if
#define error_if(test, message, ...)  if(test) { printf(message, ##__VA_ARGS__); throw std::runtime_error(message); }
#endif

} /* namespace Racecar */

#endif /* _Racecar_Racecar_h_ */

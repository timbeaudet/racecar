///
/// @file
/// @details A handful of test functions for testing the transmission component of the drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_TransmissionTest_h_
#define _Racecar_TransmissionTest_h_

namespace Racecar
{
	namespace UnitTests
	{
		bool TransmissionNeutralToFirstTest(void);

		///
		/// @details This test will put the car into gear and accelerate a little, then shift to neutral and apply the
		///   brakes. Expected: The AngularVelocity of the transmission to match the wheel at all times.
		///
		bool TransmissionBrakeInNeutraTest(void);
	};
};

#endif /* _Racecar_DifferentialTest_h_ */

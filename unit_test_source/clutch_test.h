///
/// @file
/// @details A handful of test functions for testing the clutch component of the drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_ClutchTest_h_
#define _Racecar_ClutchTest_h_

namespace Racecar
{
	namespace UnitTests
	{
		bool ClutchInputTest(void);
		bool SlippingClutchTest(void);

		///
		/// @details This test will assume that the engine and wheel are working as expected, including the engine's
		///   ability to apply a constant amount of torque when the throttle is pressed. Ensures the Clutch and Wheel
		///   do not somehow spin faster, or slower than expected.
		///
		bool EngineClutchWheelThrottleTest(void);

		///
		/// @details This test will assume that the engine and wheel are working as expected, including the wheel's
		///   ability to brake. The test will setup a simple drive-train consisting of just an Engine, Clutch and
		///   Wheel then after.
		///
		bool EngineClutchWheelBrakingTest(void);

		///
		/// @details This test will start the engine and clutch/wheel speeds at different speeds and see what happens
		///   when the clutch attempts to match speeds.
		///
		bool EngineClutchWheelMismatchTest(void);

	};
};

#endif /* _Racecar_ClutchTest_h_ */

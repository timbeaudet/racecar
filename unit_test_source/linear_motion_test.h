///
/// @file
/// @details A handful of test functions for testing the translation from angular to linear motion.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_LinearMotionTest_h_
#define _Racecar_LinearMotionTest_h_

namespace Racecar
{
	namespace UnitTests
	{
		///
		/// This test is all about taking a wheel that is on the ground, from a stopped position and applying a torque to the
		///    wheel to convert angular motion to linear motion. - Assuming infinite friction, no resistances.
		///
		bool WheelWithLinearMotion(void);

		///
		/// This test is all about taking a racecar on the ground, from a stopped position and applying a torque to the drive
		///   wheel of the car to convert angular motion to linear motion on both the wheel and the car body. - Assuming infinite
		///   friction, no resistances.
		///
		bool RacecarWithLinearMotion(void);

		///
		///
		///
		bool EngineWheelCarLinearMotion(void);
		bool EngineGearboxWheelCarLinearMotion(void);

		///
		/// Driver comes into a pit-stop, takes all four tires. The fronts and rears are on and off, and the front jack drops.
		///   The lollipop guy signals to GO, but the rear jack is stuck - rear tires hanging in the air. Driver floors the throttle
		///   rear wheels spin up to 40rad/s (approx 20-25mph). Finally the rear jack is freed and the car drops to ground with
		///   already spinning tires.
		///
		bool SpinningWheelsReleasedFromJack(void);

		///
		/// Driver is flying through the air in his rally car, he has hit the brakes to stop the wheels from spinning, though
		///   the car is still flying through the air at ~90mph (40m/s) and when it lands on the ground the wheels shall start
		///   spinning while taking some of the momentum away from the car.
		///
		bool FlyingCarHitsTrack(void);
	};
};

#endif /* _Racecar_LinearMotionTest_h_ */

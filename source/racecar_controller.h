///
/// @file
/// @details This is a simple racecar controller that provides all the controls a person would find inside a typical
///   racecar; throttle, brake, clutch pedals, steering wheel, shifter...
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_RacecarController_h_
#define _Racecar_RacecarController_h_

#include "racecar_transmission.h" //For the Racecar::Gear definitions.

namespace Racecar
{

	class RacecarControllerInterface
	{
	public:
		RacecarControllerInterface(void);
		virtual ~RacecarControllerInterface(void);

		void UpdateControls(void);

		inline float GetThrottlePosition(void) const { return mThrottlePosition; }
		inline float GetBrakePosition(void) const { return mBrakePosition; }
		inline float GetClutchPosition(void) const { return mClutchPosition; }
		inline float GetSteeringPosition(void) const { return mSteeringPosition; }
		
		//In case the user has no H-shifter, this provides a method of shifting!
		inline bool IsUpshift(void) const { return mIsUpshift; }
		inline bool IsDownshift(void) const { return mIsDownshift; }

		inline Racecar::Gear GetShifterPosition(void) const { return mShifterPosition; }

	protected:
		virtual void OnUpdateControls(void) = 0;

		inline void SetThrottlePosition(const float throttle) { mThrottlePosition = ((throttle < 0.0f) ? 0.0f : (throttle > 1.0f) ? 1.0f : throttle); }
		inline void SetBrakePosition(const float brake) { mBrakePosition = ((brake < 0.0f) ? 0.0f : (brake > 1.0f) ? 1.0f : brake); }
		inline void SetClutchPosition(const float clutch) { mClutchPosition = ((clutch < 0.0f) ? 0.0f : (clutch > 1.0f) ? 1.0f : clutch); }
		inline void SetSteeringPosition(const float steering) { mSteeringPosition = ((steering < -1.0f) ? -1.0f : (steering > 1.0f) ? 1.0f : steering); }

		inline void SetUpshift(const bool upshift) { mIsUpshift = upshift; }
		inline void SetDownshift(const bool downshift) { mIsDownshift = downshift; }
		inline void SetShifterPosition(const Gear& shifterPosition) { mShifterPosition = shifterPosition; }
	private:
		float mThrottlePosition;
		float mBrakePosition;
		float mClutchPosition;
		float mSteeringPosition;
		Gear mShifterPosition;
		bool mIsUpshift;
		bool mIsDownshift;
	};

	class DoNothingController : public RacecarControllerInterface
	{
	public:
		DoNothingController(void);
		virtual ~DoNothingController(void);

	protected:
		virtual void OnUpdateControls(void) override;
	};

	class ProgrammaticController : public RacecarControllerInterface
	{
	public:
		ProgrammaticController(void);
		virtual ~ProgrammaticController(void);

	public:
		inline void SetThrottlePosition(const float throttle) { RacecarControllerInterface::SetThrottlePosition(throttle); }
		inline void SetBrakePosition(const float brake) { RacecarControllerInterface::SetBrakePosition(brake); }
		inline void SetClutchPosition(const float clutch) { RacecarControllerInterface::SetClutchPosition(clutch); }
		inline void SetSteeringPosition(const float steering) { RacecarControllerInterface::SetSteeringPosition(steering); }
		inline void SetUpshift(const bool upshift) { RacecarControllerInterface::SetUpshift(upshift); }
		inline void SetDownshift(const bool downshift) { RacecarControllerInterface::SetDownshift(downshift); }
		inline void SetShifterPosition(const Gear shifterPosition) { RacecarControllerInterface::SetShifterPosition(shifterPosition); }

	protected:
		virtual void OnUpdateControls(void) override;
	};

};	/* namespace Racecar */

#endif /* _Racecarg_RacecarController_h_ */

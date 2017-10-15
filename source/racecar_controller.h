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
		inline bool IsUpshift(void) const { return mIsUpshift; }
		inline bool IsDownshift(void) const { return mIsDownshift; }

	protected:
		virtual void OnUpdateControls(void) = 0;

		inline void SetThrottlePosition(const float throttle) { mThrottlePosition = ((throttle < 0.0f) ? 0.0f : (throttle > 1.0f) ? 1.0f : throttle); }
		inline void SetBrakePosition(const float brake) { mBrakePosition = ((brake < 0.0f) ? 0.0f : (brake > 1.0f) ? 1.0f : brake); }
		inline void SetClutchPosition(const float clutch) { mClutchPosition = ((clutch < 0.0f) ? 0.0f : (clutch > 1.0f) ? 1.0f : clutch); }
		inline void SetSteeringPosition(const float steering) { mSteeringPosition = ((steering < -1.0f) ? -1.0f : (steering > 1.0f) ? 1.0f : steering); }

		inline void SetUpshift(const bool upshift) { mIsUpshift = upshift; }
		inline void SetDownshift(const bool downshift) { mIsDownshift = downshift; }

	private:
		float mThrottlePosition;
		float mBrakePosition;
		float mClutchPosition;
		float mSteeringPosition;
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
	protected:
		virtual void OnUpdateControls(void) override;
	};

};	/* namespace Racecar */

#endif /* _Racecarg_RacecarController_h_ */

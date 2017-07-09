///
/// @file
/// @details This is a simple racecar controller that provides all the controls a person would find inside a typical
///   racecar; throttle, brake, clutch pedals, steering wheel, shifter...
///
/// <!-- Copyright (c) 2017 Tim Beaudet - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_RacecarController_h_
#define _Racecar_RacecarController_h_

#include "../turtle_brains/math/tb_math.h"
#include "../turtle_brains/game/tb_input_action.h"

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

		inline void SetThrottlePosition(const float throttle) { mThrottlePosition = tbMath::Clamp(throttle, 0.0f, 1.0f); }
		inline void SetBrakePosition(const float brake) { mBrakePosition = tbMath::Clamp(brake, 0.0f, 1.0f); }
		inline void SetClutchPosition(const float clutch) { mClutchPosition = tbMath::Clamp(clutch, 0.0f, 1.0f); }
		inline void SetSteeringPosition(const float steering) { mSteeringPosition = tbMath::Clamp(steering, -1.0f, 1.0f); }

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

	class PlayerRacecarController : public RacecarControllerInterface
	{
	public:
		PlayerRacecarController(void);
		virtual ~PlayerRacecarController(void);

	protected:
		virtual void OnUpdateControls(void) override;

	private:
		tbGame::InputAction mThrottleAction;
		tbGame::InputAction mBrakeAction;
		tbGame::InputAction mClutchAction;
		tbGame::InputAction mTurnLeftAction;
		tbGame::InputAction mTurnRightAction;
	};

	class PlayerWheelRacecarController : public PlayerRacecarController
	{
	public:
		PlayerWheelRacecarController(void);
		virtual ~PlayerWheelRacecarController(void);

	protected:
		virtual void OnUpdateControls(void) override;
	};

};	/* namespace Racecar */

#endif /* _Racecarg_RacecarController_h_ */

///
/// @file
/// @details This is a basic simulation for a 5 speed h-shifter transmission.
///
/// <!-- Copyright (c) 2017 Tim Beaudet - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Transmission_h_
#define _Racecar_Transmission_h_

#include "rotating_body.h"

namespace Racecar
{
	enum class Gear
	{
		Reverse,
		Neutral,
		First,
		Second,
		Third,
		Fourth,
		Fifth
	};

	class RacecarControllerInterface;

	class Transmission : public RotatingBody
	{
	public:
		Transmission(void);
		virtual ~Transmission(void);

		void Simulate(const RacecarControllerInterface& racecarController);
		void SimulateShiftLogic(const RacecarControllerInterface& racecarController);

		const Gear& GetSelectedGear(void) const { return mSelectedGear; }

	protected:

	private:
		float mInputShaftSpeed;
		float mOutputShaftSpeed;
		Racecar::Gear mSelectedGear;
		bool mHasClearedShift;
	};

};	/* namespace Racecar */

#endif /* _Racecar_Transmission_h_ */

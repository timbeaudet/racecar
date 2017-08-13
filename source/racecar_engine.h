///
/// @file
/// @details This is a simple start to simulating a physical engine that simply has a torque table to look up.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Engine_h_
#define _Racecar_Engine_h_

#include "rotating_body.h"

#include <array>

namespace Racecar
{
	class RacecarControllerInterface;

	class Engine : public RotatingBody
	{
	public:
		static const size_t kTorqueTableSize = 16;

		Engine(const float momentOfInertia);
		virtual ~Engine(void);

		///
		/// @details Returns the maximum amount of torque in Nm (Newton-meters) of the engine.
		///
		float GetMaximumTorque(void) const;

		///
		/// @details Returns the maximum torque output of the engine at the given engine speed in Nm (Newton-meters).
		///
		float GetOutputTorque(const float engineSpeedRPM) const;

		///
		/// @details Returns a value from 0 to 1 representing a percentage of the maximum torque at this given engine speed.
		///
		float GetOutputValue(const float engineSpeedRPM) const;

		///
		///
		///
		void Simulate(const Racecar::RacecarControllerInterface& racecarController);

		///
		/// @details Returns the speed of the engine in revolutions per minute.
		///
		float GetEngineSpeedRPM(void) const;

	protected:

	private:
		void InitializeTorqueTableToMiata(void);

		std::array<float, kTorqueTableSize> mTorqueTable; //500, 1000 ... 8000  (normalized 0 .. 1, where 1 = maximum torque.
		float mMaximumTorque;  //In Nm
	};
};	/* namespace Racecar */

#endif /* _Racecar_Engine_h_ */

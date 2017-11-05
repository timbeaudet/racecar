///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Body_h_
#define _Racecar_Body_h_

#include "racecar.h"

#include <array>

namespace Racecar
{
	class Wheel;
	class RacecarControllerInterface;

	class RacecarBody
	{
	public:
		explicit RacecarBody(const Real& mass); //kg
		virtual ~RacecarBody(void);

		///
		///
		///
		void ControllerChange(const Racecar::RacecarControllerInterface& racecarController);

		///
		///
		///
		void Simulate(const Real fixedTime = Racecar::kFixedTimeStep);

		///
		/// @details Applies an impulse to the body of the car, which will effect the velocity immediately.
		///
		void ApplyLinearImpulse(const Real& linearImpulse);

		///
		///
		///
		void OnLinearVelocityChange(const Real& changeInLinearVelocity);

		inline const Real& GetLinearVelocity(void) const { return mLinearVelocity; }

		// This was at least needed for UnitTesting, may not be needed in API.
		void SetLinearVelocity(const Real& linearVelocity);

		inline const Real& GetMass(void) const { return mMass; }
		Real GetTotalMass(void) const;

		inline const Wheel* const GetWheel(const size_t& wheelIndex) const { return mWheels[wheelIndex]; }
		inline Wheel* GetWheel(const size_t& wheelIndex) { return mWheels[wheelIndex]; }
		inline void SetWheel(const size_t& wheelIndex, Wheel* wheelBody) { mWheels[wheelIndex] = wheelBody; }

	protected:

	private:
		std::array<Wheel*, 4> mWheels;
		Real mMass;
		Real mLinearVelocity;
	};
};	/* namespace Racecar */

#endif /* _Racecar_Body_h_ */

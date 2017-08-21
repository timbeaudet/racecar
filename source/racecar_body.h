///
/// @file
/// @details This is a simple simulation of wheel with a braking force provided by brakes on controller.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
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
		void Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real fixedTime = Racecar::kFixedTimeStep);

		///
		/// @details Applies a force to the body of the car, which will effect the accelerations for the next Simulate step.
		///
		void ApplyForce(const Real& forceInNewtons);

		///
		///
		///
		void OnApplyLinearAcceleration(const Real& changeInAcceleration);

		inline const Real& GetLinearVelocity(void) const { return mLinearVelocity; }

		// This was at least needed for UnitTesting, may not be needed in API.
		void SetLinearVelocity(const Real& linearVelocity);

		inline const Real& GetMass(void) const { return mMass; }

		inline const Wheel* const GetWheel(const size_t& wheelIndex) const { return mWheels[wheelIndex]; }
		inline Wheel* GetWheel(const size_t& wheelIndex) { return mWheels[wheelIndex]; }
		inline void SetWheel(const size_t& wheelIndex, Wheel* wheelBody) { mWheels[wheelIndex] = wheelBody; }

	protected:

	private:
		std::array<Wheel*, 4> mWheels;
		Real mMass;
		Real mLinearAcceleration;
		Real mLinearVelocity;
	};
};	/* namespace Racecar */

#endif /* _Racecar_Body_h_ */

///
/// @file
/// @details This is a basic simulation for a 5 speed h-shifter transmission.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Transmission_h_
#define _Racecar_Transmission_h_

#include "rotating_body.h"

#include <array>

namespace Racecar
{

	class GearJoint
	{
	public:
		GearJoint(Real gearRatio);
		~GearJoint(void);

		const Real& GetGearRatio(void) const { return mGearRatio; }

	private:
		const Real mGearRatio;
	};

//--------------------------------------------------------------------------------------------------------------------//

	enum class Gear
	{
		Neutral,
		First,
		Second,
		Third,
		Fourth,
		Fifth,
		Sixth,
		Reverse,
	};

	class RacecarControllerInterface;

	class Transmission : public RotatingBody
	{
	public:
		explicit Transmission(const Real momentOfInertia, const std::array<Real, 8>& gearRatios);
		virtual ~Transmission(void);

		void Simulate(const RacecarControllerInterface& racecarController, const Real& fixedTime = Racecar::kFixedTimeStep);
		void SimulateShiftLogic(const RacecarControllerInterface& racecarController);

		const Gear& GetSelectedGear(void) const { return mSelectedGear; }
		Real GetSelectedGearRatio(void) const;

		virtual Racecar::Real ComputeDownstreamInertia(const RotatingBody& fromSource) const override;
		virtual Racecar::Real ComputeUpstreamInertia(const RotatingBody& fromSource) const override;

	protected:
		virtual void OnApplyDownstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;
		virtual void OnApplyUpstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;

	private:
		Real mInputShaftSpeed;
		Real mOutputShaftSpeed;
		Racecar::Gear mSelectedGear;
		bool mHasClearedShift;

		std::array<GearJoint, 8> mGearJoints;
	};

};	/* namespace Racecar */

#endif /* _Racecar_Transmission_h_ */

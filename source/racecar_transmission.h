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

		Real ComputeTorqueImpulse(const RotatingBody& input, const RotatingBody& output, const Real& fixedTimeStep = Racecar::kFixedTimeStep);

	private:
		Real ComputeTorqueImpulseToMatchVelocity(const RotatingBody& input, const RotatingBody& output);

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
		explicit Transmission(const Real momentOfInertia, const std::array<Real, 6>& gearRatios, const Real& reverseRatio);
		virtual ~Transmission(void);

		void Simulate(const RacecarControllerInterface& racecarController, const Real& fixedTime = Racecar::kFixedTimeStep);

		const Gear& GetSelectedGear(void) const { return mSelectedGear; }
		Real GetSelectedGearRatio(void) const;

		virtual Racecar::Real ComputeDownstreamInertia(const RotatingBody& fromSource) const override;
		virtual Racecar::Real ComputeUpstreamInertia(const RotatingBody& fromSource) const override;

	protected:
		virtual void OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity, const RotatingBody& fromSource) override;
		virtual void OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity, const RotatingBody& fromSource) override;

	private:
		void SimulateShiftLogic(const RacecarControllerInterface& racecarController);

		Racecar::Gear mSelectedGear;
		bool mHasClearedShift;

		std::array<GearJoint, 8> mGearJoints;
	};

};	/* namespace Racecar */

#endif /* _Racecar_Transmission_h_ */

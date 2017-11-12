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

		const Gear& GetSelectedGear(void) const { return mSelectedGear; }
		Real GetSelectedGearRatio(void) const;

		void SetSynchromeshBox(const bool synchromeshBox) { mIsSynchromeshBox = synchromeshBox; }

		virtual Racecar::Real ComputeDownstreamInertia(void) const override;
		virtual Racecar::Real ComputeUpstreamInertia(void) const override;

	protected:
		virtual void OnControllerChange(const RacecarControllerInterface& racecarController) override;
		virtual void OnSimulate(const Real& fixedTime) override;

		virtual void OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity) override;
		virtual void OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity) override;

	private:
		Racecar::Gear mSelectedGear;
		bool mHasClearedShift;
		bool mIsSynchromeshBox;
		bool mHasUsedShifter;
		std::array<GearJoint, 8> mGearJoints;
	};

};	/* namespace Racecar */

#endif /* _Racecar_Transmission_h_ */

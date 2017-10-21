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

namespace Racecar
{

	class GearJoint
	{
	public:
		GearJoint(Real gearRatio);
		~GearJoint(void);

		Racecar::Real ComputeTorqueImpulse(const RotatingBody& input, const RotatingBody& output, const Real& fixedTimeStep = Racecar::kFixedTimeStep);
		
		void OnApplyDownstreamAcceleration(RotatingBody& input, RotatingBody& output, const Real& changeInAcceleration, const RotatingBody& fromSource);
		void OnApplyUpstreamAcceleration(RotatingBody& input, RotatingBody& output, const Real& changeInAcceleration, const RotatingBody& fromSource);

		const Real& GetGearRatio(void) const { return mGearRatio; }

	private:
		const Real mGearRatio;
	};

//--------------------------------------------------------------------------------------------------------------------//

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
		explicit Transmission(const Real momentOfInertia);
		virtual ~Transmission(void);

		void Simulate(const RacecarControllerInterface& racecarController, const Real& fixedTime = Racecar::kFixedTimeStep);
		void SimulateShiftLogic(const RacecarControllerInterface& racecarController);

		const Gear& GetSelectedGear(void) const { return mSelectedGear; }

		virtual Racecar::Real ComputeDownstreamInertia(const RotatingBody& fromSource) const override;
		virtual Racecar::Real ComputeUpstreamInertia(const RotatingBody& fromSource) const override;
		virtual void OnApplyDownstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;
		virtual void OnApplyUpstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;

	protected:
		//virtual void OnApplyDownstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;
		//virtual void OnApplyUpstreamAcceleration(const Real& changeInAcceleration, const RotatingBody& fromSource) override;

	private:
		Real mInputShaftSpeed;
		Real mOutputShaftSpeed;
		Racecar::Gear mSelectedGear;
		bool mHasClearedShift;
	};

};	/* namespace Racecar */

#endif /* _Racecar_Transmission_h_ */

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
		explicit Transmission(const Real momentOfInertia);
		virtual ~Transmission(void);

		void Simulate(const RacecarControllerInterface& racecarController, const Real& fixedTime = Racecar::kFixedTimeStep);
		void SimulateShiftLogic(const RacecarControllerInterface& racecarController);

		const Gear& GetSelectedGear(void) const { return mSelectedGear; }

		//virtual float ComputeDownstreamIntertia(const RotatingBody& fromSource) const override;
		//virtual float ComputeUpstreamInertia(const RotatingBody& fromSource) const override;

	protected:
		//virtual void OnApplyDownstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource) override;
		//virtual void OnApplyUpstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource) override;

	private:
		Real mInputShaftSpeed;
		Real mOutputShaftSpeed;
		Racecar::Gear mSelectedGear;
		bool mHasClearedShift;
	};

};	/* namespace Racecar */

#endif /* _Racecar_Transmission_h_ */

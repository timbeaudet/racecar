///
/// @file
/// @details This is a simple simulation for a clutch, that applies forces to two separate rotating bodies via
///   a frictional clutch disk.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Clutch_h_
#define _Racecar_Clutch_h_

#include "rotating_body.h"

namespace Racecar
{
	class RacecarControllerInterface;

	class ClutchJoint
	{
	public:
		ClutchJoint(Real staticFrictionCoefficient, Real kineticFrictionCoefficient);
		~ClutchJoint(void);

		inline void SetNormalForce(const Real& normalForce) { mNormalForce = normalForce; }
		Racecar::Real ComputeTorqueImpulse(const RotatingBody& input, const RotatingBody& output, const Real& fixedTimeStep = Racecar::kFixedTimeStep);

	private:
		Racecar::Real ComputeTorqueImpulseToMatchVelocity(const RotatingBody& input, const RotatingBody& output);
		Racecar::Real ComputeTorqueImpulseFromFriction(const RotatingBody& input, const RotatingBody& output, const Real& fixedTimeStep = Racecar::kFixedTimeStep);

		const Real mStaticFrictionCoefficient;
		const Real mKineticFrictionCoefficient;
		Real mNormalForce; //N
	};


	class Clutch : public RotatingBody
	{
	public:
		///
		/// @param staticFrictionCoefficient is default to 'steel-on-steel'
		/// @param kineticFrictionCoefficient is default to 'steel-on-steel'
		/// @ref http://www.school-for-champions.com/science/friction_equation.htm#.WBSr1fkrLZI
		///
		explicit Clutch(const Real& momentOfInertia, const Real& maximumNormalForce,
			const Real& staticFrictionCoefficient = 0.6, const Real& kineticFrictionCoefficient = 0.4);
		virtual ~Clutch(void);

		///
		///
		///
		Real GetClutchEngagement(void) const { return mClutchEngagement; }

	protected:

		///
		/// @details Set the clutch engagement before any Simulate calls since the Engine / other components could
		///   depend on ComputeDownstreamInertia() or ComputeUpstreamInertia() and those functions would need to know
		///   the engagement of the clutch disks, which is dependent on the RacecarControllerInterface.
		///
		virtual void OnControllerChange(const RacecarControllerInterface& racecarController) override;

		///
		///
		///
		virtual void OnSimulate(const Real& fixedTime);

		virtual Real ComputeDownstreamInertia(void) const override;
		virtual Real ComputeUpstreamInertia(void) const override;
		virtual void OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity) override;
		virtual void OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity) override;

	private:
		static Real ClutchPedalToClutchForce(const float pedalInput);
		//Real ComputeFrictionalTorque(void) const;

		Real mClutchEngagement; //0.0f for disengaged, 1.0f for completely engaged.
		const Real mMaximumNormalForce;
		ClutchJoint mClutchJoint;
	};
};	/* namespace Racecar */

#endif /* _Racecar_Clutch_h_ */

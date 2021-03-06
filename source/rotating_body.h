///
/// @file
/// @details A simple base class for each of the components of the drive-train that have rotating bodies.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_RotatingBody_h_
#define _Racecar_RotatingBody_h_

#include "racecar.h"

#include <vector>

namespace Racecar
{
	template<typename Type> constexpr Type PercentTo(const Type& value) { return value / Type(100); }

	Real RevolutionsMinuteToRadiansSecond(const Real& revolutionsMinute);
	Real RadiansSecondToRevolutionsMinute(const Real& radiansSecond);
	
	extern Real kGravityConstant;

	class RacecarControllerInterface;

	class RotatingBody
	{
	public:
		RotatingBody(const Real& momentOfInertia);
		virtual ~RotatingBody(void);

		///
		///
		///
		inline const RotatingBody* const GetInputSource(void) const { return mInputSource; }

		///
		///
		///
		void SetInputSource(RotatingBody* inputSource);

		///
		///
		///
		bool IsOutputSource(const RotatingBody& source) const;

		///
		///
		///
		size_t GetNumberOfOutputSources(void) const;

		///
		///
		///
		void AddOutputSource(RotatingBody* outputSource);
		
		///
		/// @details Should be called whenever the racecar controller changes.
		///
		void ControllerChange(const RacecarControllerInterface& racecarController);

		///
		///
		///
		void Simulate(const Real& fixedTime = kFixedTimeStep);

		///
		/// @details Applies a torque in Newton-meters, Nm, to the body.
		///
		void ApplyDownstreamAngularImpulse(const Real& angularImpulse);
		void ApplyUpstreamAngularImpulse(const Real& angularImpulse);

		///
		/// @details Returns the angular velocity of the body in degrees/second.
		///
		inline Real GetAngularVelocity(void) const { return mAngularVelocity; }

		///
		/// @details Immediately sets the angular velocity of the rotating body, no forces involved and this
		///   may ignore the angular velocities of those components upstream and downstream from this body.
		///
		void SetAngularVelocity(const Real& angularVelocity);

		inline Real GetInertia(void) const { return mInertia; }

		///
		///
		///
		virtual Real ComputeDownstreamInertia(void) const;

		///
		///
		///
		virtual Real ComputeUpstreamInertia(void) const;

	protected:
		///
		/// @details Set the inertia of the body in kg-m^2.
		///
		void SetInertia(const Real& inertia);

		virtual void OnControllerChange(const RacecarControllerInterface& racecarController);
		virtual void OnSimulate(const Real& fixedTime);

		///
		/// @details Changes the bodies acceleration.
		///
		/// @param changeInAcceleration should be in deg/sec/sec
		///
	
		//TODO: DriveTrain: FIX: HACK: These methods should be protected, they are not so that the differential can work.
	public: virtual void OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity);
	public: virtual void OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity);
	protected:
		///
		/// @details Returns the input source for the rotating body, in a way that forces can be transmitted back.
		///
		const RotatingBody& GetExpectedInputSource(void) const;
		RotatingBody& GetExpectedInputSource(void);
		inline RotatingBody* GetInputSource(void) { return mInputSource; }
		
		const RotatingBody& GetExpectedOutputSource(const size_t& sourceIndex) const;
		RotatingBody& GetExpectedOutputSource(const size_t& sourceIndex);
		const std::vector<RotatingBody*>& GetOutputSources(void) const;
		std::vector<RotatingBody*>& GetOutputSources(void);

	private:
		RotatingBody* mInputSource;
		std::vector<RotatingBody*> mOutputSources;

		Real mInertia;             //Rotating inertia of all components in engine include flywheel and pressure plate.
		Real mAngularVelocity;     //Radians / Second
	};
};	/* namespace Racecar */

#endif /* _Racecar_RotatingBody_h_ */

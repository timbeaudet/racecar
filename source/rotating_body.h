///
/// @file
/// @details A simple base class for each of the components of the drivetrain that have rotating bodies.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_RotatingBody_h_
#define _Racecar_RotatingBody_h_

#include "racecar.h"

#include <vector>

namespace Racecar
{
	template<typename Type> constexpr Type PercentTo(const Type& value) { return value / Type(100); }

	///
	/// @details Computes the moment of inertia for a solid disk using imperial measurements for input and the output
	///   will be in metric units (kg-m^2).
	///
	/// @note The output will be in metric units, kg-m^2.
	///
	Real ComputeInertiaImperial(Real massInPounds, Real radiusInInches);

	Real RevolutionsMinuteToDegreesSecond(const Real revolutionsMinute);
	Real DegreesSecondToRevolutionsMinute(const Real degreesSecond);
	
	class RotatingBody
	{
	public:
		RotatingBody(const Real momentOfInertia);
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
		void AddOutputSource(RotatingBody* outputSource);

		///
		///
		///
		void Simulate(const Real fixedTime = kFixedTimeStep);

		///
		/// @details Applies a torque in Newton-meters, Nm, to the body.
		///
		void ApplyDownstreamTorque(const Real torqueNewtonMeters, const RotatingBody& fromSource);
		void ApplyUpstreamTorque(const Real torqueNewtonMeters, const RotatingBody& fromSource);

		///
		/// @details Returns the angular velocity of the body in degrees/second.
		///
		inline Real GetAngularVelocity(void) const { return mAngularVelocity; }

		inline Real GetInertia(void) const { return mInertia; }

		///
		///
		///
		virtual Real ComputeDownstreamInertia(const RotatingBody& fromSource) const;

		///
		///
		///
		virtual Real ComputeUpstreamInertia(const RotatingBody& fromSource) const;

	protected:
		///
		/// @details Set the inertia of the body in kg-m^2.
		///
		void SetInertia(const Real inertia);

		void SetAngularVelocity(const Real angularVelocity);
		void AddAngularAcceleration(const Real angularAcceleration) { mAngularAcceleration += angularAcceleration; }

		///
		/// @details Changes the bodies acceleration.
		///
		/// @param changeInAcceleration should be in deg/sec/sec
		///
	
		//TODO: FIX: HACK: These methods should be protected, they are not so that the differential can work.
	public:	virtual void OnApplyDownstreamAcceleration(const Real changeInAcceleration, const RotatingBody& fromSource);
	public: virtual void OnApplyUpstreamAcceleration(const Real changeInAcceleration, const RotatingBody& fromSource);
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
		//bool mLockedWithInput;      //Potential future use to replace virtual Down/Up stream compute/accelerates.

		void SetAngularAcceleration(const Real angularAcceleration);

		Real mInertia;             //Rotating inertia of all components in engine include flywheel and pressure plate.
		Real mAngularAcceleration; //Degrees / Second / Second
		Real mAngularVelocity;     //Degrees / Second
	};
};	/* namespace Racecar */

#endif /* _Racecar_RotatingBody_h_ */

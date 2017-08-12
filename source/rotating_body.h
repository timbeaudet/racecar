///
/// @file
/// @details A simple base class for each of the components of the drivetrain that have rotating bodies.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_RotatingBody_h_
#define _Racecar_RotatingBody_h_

#include <vector>

namespace Racecar
{
	template<typename Type> constexpr Type PercentTo(const Type& value) { return value / Type(100); }
	float ComputeInertia(float massInPounds, float radiusInInches);
	float RevolutionsMinuteToDegreesSecond(const float revolutionsMinute);
	float DegreesSecondToRevolutionsMinute(const float degreesSecond);
	
	class RotatingBody
	{
	public:
		RotatingBody(void);
		virtual ~RotatingBody(void);

		///
		///
		///
		void SetInput(RotatingBody* input);

		///
		///
		///
		inline const RotatingBody* const GetInput(void) const { return mInput; }

		///
		///
		///
		void AddOutput(RotatingBody* output);

		///
		///
		///
		void Simulate(void);

		///
		/// @details Applies a torque in Newton-meters, Nm, to the body.
		///
		void ApplyDownstreamTorque(const float torqueNewtonMeters, const RotatingBody& fromSource);
		void ApplyUpstreamTorque(const float torqueNewtonMeters, const RotatingBody& fromSource);

		///
		/// @details Returns the angular velocity of the body in degrees/second.
		///
		inline float GetAngularVelocity(void) const { return mAngularVelocity; }

		inline float GetInertia(void) const { return mInertia; }

		///
		///
		///
		virtual float ComputeDownstreamInertia(const RotatingBody& fromSource) const;

		///
		///
		///
		virtual float ComputeUpstreamInertia(const RotatingBody& fromSource) const;

	protected:
		///
		/// @details Set the inertia of the body in kg-m.
		///
		void SetInertia(const float inertia);

		void SetAngularVelocity(const float angularVelocity);
		void AddAngularAcceleration(const float angularAcceleration) { mAngularAcceleration += angularAcceleration; }

		///
		/// @details Changes the bodies acceleration.
		///
		/// @param changeInAcceleration should be in deg/sec/sec
		///
		virtual void OnApplyDownstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource);
		virtual void OnApplyUpstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource);

		///
		/// @details Returns the input source for the rotating body, in a way that forces can be transmitted back.
		///
		const RotatingBody& GetExpectedInputSource(void) const;
		RotatingBody& GetExpectedInputSource(void);

		const RotatingBody& GetExpectedOutputSource(const size_t& sourceIndex) const;
		RotatingBody& GetExpectedOutputSource(const size_t& sourceIndex);
		const std::vector<RotatingBody*>& GetOutputs(void) const;
		std::vector<RotatingBody*>& GetOutputs(void);

	private:
		RotatingBody* mInput;
		std::vector<RotatingBody*> mOutputs;
		//bool mLockedWithInput;      //Potential future use to replace virtual Down/Up stream compute/accelerates.

		void SetAngularAcceleration(const float angularAcceleration);

		float mInertia;             //Rotating inertia of all components in engine include flywheel and pressure plate.
		float mAngularAcceleration; //Degrees / Second / Second
		float mAngularVelocity;     //Degrees / Second
	};
};	/* namespace Racecar */

#endif /* _Racecar_RotatingBody_h_ */

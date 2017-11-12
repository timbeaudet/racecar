///
/// @file
/// @details This is a simple start to simulating a physical engine that simply has a torque table to look up.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_Engine_h_
#define _Racecar_Engine_h_

#include "rotating_body.h"

#include <array>

namespace Racecar
{
	class RacecarControllerInterface;

	class ConstantEngine : public RotatingBody
	{
	public:
		explicit ConstantEngine(const Real& momentOfInertia, const Real& constantTorque, const Real& resistanceTorque);
		virtual ~ConstantEngine(void);

		Real GetEngineSpeedRPM(void) const;

	protected:
		virtual void OnControllerChange(const Racecar::RacecarControllerInterface& racecarController) override;
		virtual void OnSimulate(const Real& fixedTime) override;

	private:
		const Real mConstantTorque;     //Applied if throttle is greater than 0.5
		const Real mResistanceTorque;   //Applied if throttle is less than 0.1, and angular velocity > 0.
		float mThrottlePosition;
	};

//--------------------------------------------------------------------------------------------------------------------//

	class TorqueCurve
	{
	public:
		static TorqueCurve MiataTorqueCurve(void);

		TorqueCurve(void);
		~TorqueCurve(void);

		///
		/// @details Inserts a point for the curve to follow a more realistic torque/power curve of an internal combustion engine.
		///
		/// @param engineSpeedRPM Must be a positive value representing the speed of the engine in revolutions-per-minute.
		/// @param torque Must be a positive value representing the torque produced at engineSpeedRPM.
		///
		/// @note Cannot be called once the TorqueCurve object has been normalized or an error condition will be triggered.
		///
		void AddPlotPoint(const Real engineSpeedRPM, const Real torque);

		///
		/// @details Finds the maximum torque value in the table and normalizes all values to be within 0.0 to 1.0.
		///
		void NormalizeTorqueCurve(void);

		///
		/// @details Will return true if the TorqueTable has been normalized, "set in stone."
		///
		inline bool IsNormalized(void) const { return mIsNormalized; }

		///
		/// @details Returns the maximum amount of torque in Nm (Newton-meters) of the engine.
		///
		Real GetMaximumTorque(void) const;

		///
		/// @details Returns the maximum torque output of the engine at the given engine speed in Nm (Newton-meters).
		///
		Real GetOutputTorque(const Real engineSpeedRPM) const;

		///
		/// @details Returns a value from 0 to 1 representing a percentage of the maximum torque at this given engine speed.
		///
		Real GetOutputValue(const Real engineSpeedRPM) const;

	private:
		static const size_t kTorqueTableSize = 16;

		typedef std::pair<Real, Real> PlotPoint; //RPM, NormalizedTorque
		std::vector<PlotPoint> mTorqueTable;
		Real mMaximumTorque;  //In Nm
		bool mIsNormalized;
	};

//--------------------------------------------------------------------------------------------------------------------//

	class Engine : public RotatingBody
	{
	public:
		explicit Engine(const Real& momentOfInertia, const TorqueCurve& torqueCurve);
		virtual ~Engine(void);

		///
		/// @details Returns the speed of the engine in revolutions per minute.
		///
		Real GetEngineSpeedRPM(void) const;

		///
		/// @note This defaults to 0.0
		///
		void SetEngineFrictionResistance(const Real& resistance);

		///
		/// @details Sets the minimum speed the engine will try forcing itself to. This creates a FICTIONAL force and is
		///   NOT SIMULATION QUALITY. This would prevent the engine from stalling below some point in rpm range.
		///
		/// @param speedRadiansPerSecond The speed at which the engine will begin applying this fictional force if the
		///   engine speed is below. Radians / Second.
		///
		/// @note If this is set to below zero (-1.0) the engine will not perform the fictional force. This is default behavior.
		///
		void SetMinimumEngineSpeed(const Real& speedRadiansPerSecond);

		///
		/// @details Sets the maximums speed the engine will rotate at before getting an ignition cut, such as red-lining.
		///   This may not be simulation quality, but prevents the engine from endlessly getting faster.
		///
		/// @param speedRadiansPerSecond The speed at which the engine will not apply any acceleration from throttle when
		///   engine speed is already above. Radians / Second.
		///
		/// @note If this is set to below zero (-1.0) the engine will not perform the throttle-cut. This is default behavior.
		///
		void SetMaximumEngineSpeed(const Real& speedRadiansPerSecond);

		///
		/// @details Changes the behavior of the engine to output a constant or non-constant amount of power, essentially if
		///   this is set to false (default) the engine will produce the angular impulse from the torque supplied by TorqueCurve, 
		///   otherwise it will produce an angular impulse based on the rotational speed and TorqueCurve.
		///
		void SetConstantPower(const bool constantPower);

	protected:
		virtual void OnControllerChange(const Racecar::RacecarControllerInterface& racecarController) override;
		virtual void OnSimulate(const Real& fixedTime);

	private:
		const TorqueCurve mTorqueCurve;
		Real mFrictionResistance;
		Real mMinimumEngineSpeed;
		Real mMaximumEngineSpeed;
		float mThrottlePosition;
		bool mConstantPower;
	};
};	/* namespace Racecar */

#endif /* _Racecar_Engine_h_ */

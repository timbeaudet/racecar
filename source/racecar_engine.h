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

		void Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real& fixedTime = Racecar::kFixedTimeStep);

		Real GetEngineSpeedRPM(void) const;

	private:
		const Real mConstantTorque;     //Applied if throttle is greater than 0.5
		const Real mResistanceTorque;   //Applied if throttle is less than 0.1, and angular velocity > 0.
	};

//--------------------------------------------------------------------------------------------------------------------//

	class TorqueCurve
	{
	public:
		TorqueCurve(void);
		~TorqueCurve(void);

		static TorqueCurve MiataTorqueCurve(void);

		///
		///
		///
		void AddPlotPoint(const Real engineSpeedRPM, const Real torque);

		///
		///
		///
		void NormalizeTorqueCurve(void);


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
		///
		///
		void Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real& fixedTime = Racecar::kFixedTimeStep);

		///
		/// @details Returns the speed of the engine in revolutions per minute.
		///
		Real GetEngineSpeedRPM(void) const;

	protected:

	private:
		const TorqueCurve mTorqueCurve;
	};
};	/* namespace Racecar */

#endif /* _Racecar_Engine_h_ */

///
/// @file
/// @details This is a simple start to simulating a physical engine that simply has a torque table to look up.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_engine.h"
#include "racecar_controller.h"

//-------------------------------------------------------------------------------------------------------------------//

Racecar::ConstantEngine::ConstantEngine(const Real& momentOfInertia, const Real& constantTorque, const Real& resistanceTorque) :
	RotatingBody(momentOfInertia),
	mConstantTorque(constantTorque),
	mResistanceTorque(resistanceTorque),
	mThrottlePosition(0.0)
{
	error_if(mResistanceTorque < 0.0, "Then engine resistance torque should always be >= 0.")
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::ConstantEngine::~ConstantEngine(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//


//-------------------------------------------------------------------------------------------------------------------//

void Racecar::ConstantEngine::OnControllerChange(const Racecar::RacecarControllerInterface& racecarController)
{
	mThrottlePosition = racecarController.GetThrottlePosition();
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::ConstantEngine::OnSimulate(const Real& fixedTime)
{
	const Real revolutions(GetEngineSpeedRPM() / 60.0 * fixedTime);

	if (mThrottlePosition > 0.5)
	{
		ApplyDownstreamAngularImpulse(mConstantTorque * fixedTime);
	}
	else if (mThrottlePosition < 0.1 && mResistanceTorque > kEpsilon)
	{
		const Real totalInertia(ComputeDownstreamInertia());
		const Real maximumImpulse(totalInertia * GetAngularVelocity()); //kg*m^2 / s
		const Real actualImpulse(mResistanceTorque * fixedTime); //kg*m^2 / s
		const Real appliedImpulse((actualImpulse > maximumImpulse) ? maximumImpulse : actualImpulse);
		//The /fixedTime is to apply this as an impulse, deeper down it will *fixedTime.
		ApplyDownstreamAngularImpulse(appliedImpulse * -Racecar::Sign(GetAngularVelocity()));
	}

	//Now that all torques have been applied to the engine, step it forward in time.
	RotatingBody::OnSimulate(fixedTime);
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::ConstantEngine::GetEngineSpeedRPM(void) const
{
	return Racecar::RadiansSecondToRevolutionsMinute(GetAngularVelocity());
}

//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//

Racecar::TorqueCurve Racecar::TorqueCurve::MiataTorqueCurve(void)
{	//http://www.automobile-catalog.com/curve/1999/1667030/mazda_mx-5_1_9.html
	TorqueCurve curve;
	curve.AddPlotPoint(500, 25.0);
	curve.AddPlotPoint(1000, 75.0);
	curve.AddPlotPoint(1500, 112.0);
	curve.AddPlotPoint(2000, 130.0);
	curve.AddPlotPoint(2500, 137.0);
	curve.AddPlotPoint(3000, 150.0);
	curve.AddPlotPoint(3500, 155.0);
	curve.AddPlotPoint(4000, 158.0);
	curve.AddPlotPoint(4500, 162.0);
	curve.AddPlotPoint(5000, 160.0);
	curve.AddPlotPoint(5500, 159.0);
	curve.AddPlotPoint(6000, 156.5);
	curve.AddPlotPoint(6500, 151.0);
	curve.AddPlotPoint(7000, 127.0);
	curve.AddPlotPoint(7500, 25.0);
	curve.AddPlotPoint(8000, 0.0);
	curve.NormalizeTorqueCurve();
	return curve;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::TorqueCurve::TorqueCurve(void) :
	mTorqueTable(),
	mMaximumTorque(0.0),
	mIsNormalized(false)
{
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::TorqueCurve::~TorqueCurve(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::TorqueCurve::AddPlotPoint(const Racecar::Real engineSpeedRPM, const Racecar::Real torque)
{
	error_if(true == mIsNormalized, "Cannot add more plot points to a table that is already normalized.");
	error_if(engineSpeedRPM < 0.0, "Cannot add plot point for engine speeds less than zero.");
	error_if(torque < 0.0, "Cannot add plot point for torque amounts that are less than zero.");

	auto findItr = std::find_if(mTorqueTable.begin(), mTorqueTable.end(), [engineSpeedRPM](PlotPoint& pt) { return fabs(pt.first - engineSpeedRPM) < 0.1; });
	error_if(mTorqueTable.end() != findItr, "Cannot plot a point on top of another point!");

	PlotPoint pt(engineSpeedRPM, torque);
	mTorqueTable.push_back(pt);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::TorqueCurve::NormalizeTorqueCurve(void)
{
	error_if(true == mTorqueTable.empty(), "Cannot normalize a table without plotted points. Call AddPlotPoint() to make it interesting.");

	std::sort(mTorqueTable.begin(), mTorqueTable.end(), [](PlotPoint& a, PlotPoint& b) { return a.second < b.second; });
	mMaximumTorque = mTorqueTable.back().second;

	std::sort(mTorqueTable.begin(), mTorqueTable.end(), [](PlotPoint& a, PlotPoint& b) { return a.first < b.first; });
	std::for_each(mTorqueTable.begin(), mTorqueTable.end(), [this](PlotPoint& pt) { pt.second /= mMaximumTorque; });

	mIsNormalized = true;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::TorqueCurve::GetMaximumTorque(void) const
{
	return mMaximumTorque;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::TorqueCurve::GetOutputTorque(const Real engineSpeedRPM) const
{
	return GetOutputValue(engineSpeedRPM) * mMaximumTorque;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::TorqueCurve::GetOutputValue(const Real engineSpeedRPM) const
{
	error_if(false == mIsNormalized, "Cannot get output of a TorqueCurve that has not been normalized. Call NormalizeTorqueCurve().");

	PlotPoint previousPoint = mTorqueTable.front();
	if (engineSpeedRPM < previousPoint.first)
	{	//The RPM of the engine is lower than the lowest in torque table.
		return previousPoint.second;
	}

	for (size_t index(1); index < kTorqueTableSize; ++index)
	{
		const PlotPoint& currentPoint(mTorqueTable[index]);
		const Real& currentRPM(currentPoint.first);
		const Real& currentTorque(currentPoint.second);

		if (engineSpeedRPM > currentRPM)
		{
			previousPoint = currentPoint;
			continue;
		}

		const Real& previousRPM(previousPoint.first);
		const Real& previousTorque(previousPoint.second);
		const Real percentage = 1.0 - ((currentRPM - engineSpeedRPM) / (currentRPM - previousRPM));
		return previousTorque + ((currentTorque - previousTorque) * percentage);
	}

	warning_if(true, "Value not found for RPM: %f in torque table.", engineSpeedRPM);
	return mTorqueTable.back().second;
}

//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//

Racecar::Engine::Engine(const Real& momentOfInertia, const TorqueCurve& torqueCurve) :
	RotatingBody(momentOfInertia),
	mTorqueCurve(torqueCurve),
	mFrictionResistance(0.0),
	mMinimumEngineSpeed(-1.0),
	mMaximumEngineSpeed(-1.0),
	mThrottlePosition(0.0f),
	mConstantPower(true)
{
	error_if(false == mTorqueCurve.IsNormalized(), "Engine expects the TorqueCurve to be normalized / finalized.");
	SetAngularVelocity(Racecar::RevolutionsMinuteToRadiansSecond(1000.0));
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Engine::~Engine(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Engine::OnControllerChange(const Racecar::RacecarControllerInterface& racecarController)
{
	mThrottlePosition = racecarController.GetThrottlePosition();
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Engine::OnSimulate(const Real& fixedTime)
{
	if (mMaximumEngineSpeed < 0.0 || GetAngularVelocity() < mMaximumEngineSpeed)
	{
		const Real onThrottleTorque(mTorqueCurve.GetOutputTorque(GetEngineSpeedRPM()) * mThrottlePosition);
		const Real appliedEngineTorque(onThrottleTorque);
//		ApplyDownstreamAngularImpulse(appliedEngineTorque * fixedTime);

		if (GetAngularVelocity() < 1.0 || true == mConstantPower)
		{
			ApplyDownstreamAngularImpulse(appliedEngineTorque);
		}
		else
		{	//Power = Work / Time  which is  Nm / s    engineTorque is in Nm,  AngularVel is rad/s (or 1/s)
			const Real power = appliedEngineTorque * (GetAngularVelocity());  //Nm * rad/s = kg*m^2/s^3
			const Real work = power * fixedTime;                              //Work = Nm,  Power * time = kg*m^2/s^3 * s = kg*m^2/s^2 = Nm
			ApplyDownstreamAngularImpulse(work * fixedTime);                  //Finally Nm * time = impulse
		}
	}
	
	//if (mThrottlePosition < 0.01)
	{
		const Real engineResistanceTorque(GetAngularVelocity() * mFrictionResistance);
		ApplyDownstreamAngularImpulse(-engineResistanceTorque * fixedTime);
	}

	//Now that all torques have been applied to the engine, step it forward in time.
	RotatingBody::OnSimulate(fixedTime);

	//
	//Create a fictional force to keep the engine from stalling out. This is NOT simulation quality here...
	//          divide 2pi to go to rev, multiply 60 to go min.
	if (mMinimumEngineSpeed > 0.0)
	{
		const Real differenceTo1000((GetAngularVelocity() - mMinimumEngineSpeed) / 6.28 * 60);
		if (differenceTo1000 < 0.0)
		{
			const Real totalInertia(ComputeDownstreamInertia());
			ApplyDownstreamAngularImpulse(-differenceTo1000 * fixedTime * totalInertia);
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Engine::GetEngineSpeedRPM(void) const
{
	return Racecar::RadiansSecondToRevolutionsMinute(GetAngularVelocity());
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Engine::SetEngineFrictionResistance(const Real& frictionResistance)
{
	error_if(frictionResistance < 0.0, "Expected resistance to be a positive value.");
	mFrictionResistance = frictionResistance;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Engine::SetMinimumEngineSpeed(const Real& speedRadiansPerSecond)
{
	mMinimumEngineSpeed = speedRadiansPerSecond;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Engine::SetMaximumEngineSpeed(const Real& speedRadiansPerSecond)
{
	mMaximumEngineSpeed = speedRadiansPerSecond;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Engine::SetConstantPower(const bool constantPower)
{
	mConstantPower = constantPower;
}

//-------------------------------------------------------------------------------------------------------------------//

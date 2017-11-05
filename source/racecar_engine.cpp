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
	mResistanceTorque(resistanceTorque)
{
	error_if(mResistanceTorque < 0.0, "Then engine resistance torque should always be >= 0.")
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::ConstantEngine::~ConstantEngine(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::ConstantEngine::Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real& fixedTime)
{
	const Real revolutions(GetEngineSpeedRPM() / 60.0 * fixedTime);

	if (racecarController.GetThrottlePosition() > 0.5)
	{
		ApplyDownstreamAngularImpulse(mConstantTorque * fixedTime);
	}
	else if (racecarController.GetThrottlePosition() < 0.1 && mResistanceTorque > kEpsilon)
	{
		const Real totalInertia(ComputeDownstreamInertia());
		const Real maximumImpulse(totalInertia * GetAngularVelocity()); //kg*m^2 / s
		const Real actualImpulse(mResistanceTorque * fixedTime); //kg*m^2 / s
		const Real appliedImpulse((actualImpulse > maximumImpulse) ? maximumImpulse : actualImpulse);
		//The /fixedTime is to apply this as an impulse, deeper down it will *fixedTime.
		ApplyDownstreamAngularImpulse(appliedImpulse * -Racecar::Sign(GetAngularVelocity()));
	}

	//Now that all torques have been applied to the engine, step it forward in time.
	RotatingBody::Simulate(fixedTime);
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::ConstantEngine::GetEngineSpeedRPM(void) const
{
	return Racecar::RadiansSecondToRevolutionsMinute(GetAngularVelocity());
}

//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
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

	auto findItr = std::find_if(mTorqueTable.begin(), mTorqueTable.end(), [engineSpeedRPM](PlotPoint& pt) { return fabs(pt.first - engineSpeedRPM) < kEpsilon; });
	error_if(mTorqueTable.end() != findItr, "Cannot plot a point on top of another point!");

//	for (const PlotPoint& pt : mTorqueTable) { if (pt.first )

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
}

//-------------------------------------------------------------------------------------------------------------------//

//void Racecar::TorqueCurve::InitializeTorqueTableToMiata(void)
Racecar::TorqueCurve Racecar::TorqueCurve::MiataTorqueCurve(void)
{	//http://www.automobile-catalog.com/curve/1999/1667030/mazda_mx-5_1_9.html
	//mTorqueTable[0] = 25.0;   //500rpm
	//mTorqueTable[1] = 75.0;   //1000rpm
	//mTorqueTable[2] = 112.0;
	//mTorqueTable[3] = 130.0;  //2000rpm
	//mTorqueTable[4] = 137.0;
	//mTorqueTable[5] = 150.0;  //3000rpm
	//mTorqueTable[6] = 155.0;
	//mTorqueTable[7] = 158.0;  //4000rpm
	//mTorqueTable[8] = 162.0;
	//mTorqueTable[9] = 160.0;  //5000rpm
	//mTorqueTable[10] = 159.0;
	//mTorqueTable[11] = 156.5; //6000rpm
	//mTorqueTable[12] = 151.0;
	//mTorqueTable[13] = 127.0; //7000rpm
	//mTorqueTable[14] = 25.0;
	//mTorqueTable[15] = 0.0;  //8000rpm

	//error_if(mTorqueTable.size() != 16, "Error: Expected table size to be 16, may need to change step size or something.");
	//error_if(mTorqueTable.size() != kTorqueTableSize, "Error: Expected table size to be 16, may need to change step size or something.");

	//mMaximumTorque = 162.0; //If unknown a search could find it...
	//for (size_t index(0); index < mTorqueTable.size(); ++index)
	//{	//Normalize the curve so it fits in range 0 to 1, from no to max torque??
	//	mTorqueTable[index] /= mMaximumTorque;
	//}

	TorqueCurve curve;
	curve.AddPlotPoint(500,  25.0);
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
	return curve;
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
	for (size_t index(0); index < kTorqueTableSize; ++index)
	{
		const PlotPoint& currentPoint(mTorqueTable[index]);
		const Real& currentRPM(currentPoint.first);
		const Real& currentTorque(currentPoint.second);

		if (engineSpeedRPM > currentRPM)
		{
			previousPoint = currentPoint;
			continue;
		}

		const Real& previousTorque(previousPoint.second);
		const Real percentage = 1.0 - ((currentRPM - engineSpeedRPM) / 500.0);		
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
	mTorqueCurve(torqueCurve)
{
	SetAngularVelocity(Racecar::RevolutionsMinuteToRadiansSecond(1000.0));
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Engine::~Engine(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Engine::Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real& fixedTime)
{
	if (GetEngineSpeedRPM() < 6500)
	{
		const Real minimumIdleTorque(5.2 * 1.3558179); //ft-lbs to Nm
		const Real onThrottleTorque(mTorqueCurve.GetOutputTorque(GetEngineSpeedRPM()) * racecarController.GetThrottlePosition());
		const Real appliedEngineTorque((minimumIdleTorque < onThrottleTorque) ? onThrottleTorque : minimumIdleTorque);
		ApplyDownstreamAngularImpulse(appliedEngineTorque * fixedTime);
	}

	//Resistance of 1Nm for every 32 rad/s <-- THIS COMMENT MIGHT NOT BE TRUE ANYMORE...
	const Real engineResistanceTorque(GetAngularVelocity() * 0.0625);
	ApplyDownstreamAngularImpulse(-engineResistanceTorque * fixedTime);

	//Now that all torques have been applied to the engine, step it forward in time.
	RotatingBody::Simulate(fixedTime);

	//
	//Create a fictional force to keep the engine from stalling out. This is NOT simulation quality here...
	//
	const Real differenceTo1000(GetEngineSpeedRPM() - 1000.0); //Why does this work <<< and this vvv doesn't?
	//const Real differenceTo1000(GetAngularVelocity() - Racecar::RevolutionsMinuteToRadiansSecond(1000.0));
	if (differenceTo1000 < 0.0)
	{
		const Real totalInertia(ComputeDownstreamInertia());
		ApplyDownstreamAngularImpulse(-differenceTo1000 * fixedTime * totalInertia);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Engine::GetEngineSpeedRPM(void) const
{
	return Racecar::RadiansSecondToRevolutionsMinute(GetAngularVelocity());
}

//-------------------------------------------------------------------------------------------------------------------//

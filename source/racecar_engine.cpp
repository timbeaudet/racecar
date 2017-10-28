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
		ApplyDownstreamAngularImpulse(mConstantTorque * fixedTime, *this);
	}
	if (racecarController.GetThrottlePosition() < 0.1 && mResistanceTorque > kElipson)
	{
		const Real totalInertia(ComputeDownstreamInertia(*this));
		const Real maximumImpulse(totalInertia * GetAngularVelocity()); //kg*m^2 / s
		const Real actualImpulse(mResistanceTorque * fixedTime); //kg*m^2 / s
		const Real appliedImpulse((actualImpulse > maximumImpulse) ? maximumImpulse : actualImpulse);
		//The /fixedTime is to apply this as an impulse, deeper down it will *fixedTime.
		ApplyDownstreamAngularImpulse(appliedImpulse * -Racecar::Sign(GetAngularVelocity()), *this);
	}

	{	//Resistance of 1Nm for every 32 rad/s <-- THIS COMMENT MIGHT NOT BE TRUE ANYMORE...
//		const Real engineResistanceTorque(GetAngularVelocity() * 0.0625);
//		ApplyDownstreamTorque(-engineResistanceTorque, *this);
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

Racecar::Engine::Engine(const Real& momentOfInertia) :
	RotatingBody(momentOfInertia),
	mTorqueTable(),
	mMaximumTorque(162.0)
{
	InitializeTorqueTableToMiata();

	SetAngularVelocity(Racecar::RevolutionsMinuteToRadiansSecond(1000.0));
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Engine::~Engine(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Engine::InitializeTorqueTableToMiata(void)
{	//http://www.automobile-catalog.com/curve/1999/1667030/mazda_mx-5_1_9.html
	mTorqueTable[0] = 25.0;   //500rpm
	mTorqueTable[1] = 75.0;   //1000rpm
	mTorqueTable[2] = 112.0;
	mTorqueTable[3] = 130.0;  //2000rpm
	mTorqueTable[4] = 137.0;
	mTorqueTable[5] = 150.0;  //3000rpm
	mTorqueTable[6] = 155.0;
	mTorqueTable[7] = 158.0;  //4000rpm
	mTorqueTable[8] = 162.0;
	mTorqueTable[9] = 160.0;  //5000rpm
	mTorqueTable[10] = 159.0;
	mTorqueTable[11] = 156.5; //6000rpm
	mTorqueTable[12] = 151.0;
	mTorqueTable[13] = 127.0; //7000rpm
	mTorqueTable[14] = 25.0;
	mTorqueTable[15] = 0.0;  //8000rpm

	error_if(mTorqueTable.size() != 16, "Error: Expected table size to be 16, may need to change step size or something.");
	error_if(mTorqueTable.size() != kTorqueTableSize, "Error: Expected table size to be 16, may need to change step size or something.");

	mMaximumTorque = 162.0; //If unknown a search could find it...
	for (size_t index(0); index < mTorqueTable.size(); ++index)
	{	//Normalize the curve so it fits in range 0 to 1, from no to max torque??
		mTorqueTable[index] /= mMaximumTorque;
	}
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Engine::GetMaximumTorque(void) const
{
	return mMaximumTorque;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Engine::GetOutputTorque(const Real engineSpeedRPM) const
{
	return GetOutputValue(engineSpeedRPM) * mMaximumTorque;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Engine::GetOutputValue(const Real engineSpeedRPM) const
{
	Real previous = mTorqueTable.front();
	for (size_t index(0); index < kTorqueTableSize; ++index)
	{
		Real currentRPM((index) * 500.0);
		if (engineSpeedRPM > currentRPM)
		{
			previous = mTorqueTable[index];
			continue;
		}

		//0.2 = (1500 - 1400) / 500.0f

		Real percentage = 1.0 - ((currentRPM - engineSpeedRPM) / 500.0);
		return previous + ((mTorqueTable[index] - previous) * percentage);
	}

	return mTorqueTable.back();
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::Engine::Simulate(const Racecar::RacecarControllerInterface& racecarController, const Real& fixedTime)
{
	if (GetEngineSpeedRPM() < 6500)
	{
		const Real minimumIdleTorque(5.2 * 1.3558179); //ft-lbs to Nm
		const Real onThrottleTorque(GetOutputTorque(GetEngineSpeedRPM()) * racecarController.GetThrottlePosition());
		const Real appliedEngineTorque((minimumIdleTorque < onThrottleTorque) ? onThrottleTorque : minimumIdleTorque);
		ApplyDownstreamAngularImpulse(appliedEngineTorque * fixedTime, *this);
	}

	//Resistance of 1Nm for every 32 rad/s <-- THIS COMMENT MIGHT NOT BE TRUE ANYMORE...
	const Real engineResistanceTorque(GetAngularVelocity() * 0.0625);
	ApplyDownstreamAngularImpulse(-engineResistanceTorque * fixedTime, *this);

	//Now that all torques have been applied to the engine, step it forward in time.
	RotatingBody::Simulate(fixedTime);

	//
	//Create a fictional force to keep the engine from stalling out. This is NOT simulation quality here...
	//
	const Real differenceTo1000(GetEngineSpeedRPM() - 1000.0); //Why does this work <<< and this vvv doesn't?
	//const Real differenceTo1000(GetAngularVelocity() - Racecar::RevolutionsMinuteToRadiansSecond(1000.0));
	if (differenceTo1000 < 0.0)
	{
		const Real totalInertia(ComputeDownstreamInertia(*this));
		ApplyDownstreamAngularImpulse(-differenceTo1000 * fixedTime * totalInertia, *this);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Engine::GetEngineSpeedRPM(void) const
{
	return Racecar::RadiansSecondToRevolutionsMinute(GetAngularVelocity());
}

//-------------------------------------------------------------------------------------------------------------------//

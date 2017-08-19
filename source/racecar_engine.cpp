///
/// @file
/// @details This is a simple start to simulating a physical engine that simply has a torque table to look up.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "racecar_engine.h"
#include "racecar_controller.h"

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Engine::Engine(const Real momentOfInertia) :
	RotatingBody(momentOfInertia),
	mTorqueTable(),
	mMaximumTorque(162.0)
{
	InitializeTorqueTableToMiata();

	SetAngularVelocity(360.0 / 60 * 1000);
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

	error_if(kTorqueTableSize != 16, "Error: Expected table size to be 16, may need to change step size or something.");

	mMaximumTorque = 162.0; //If unknown a search could find it...
	for (size_t index(0); index < kTorqueTableSize; ++index)
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
	const Real revolutions(GetEngineSpeedRPM() / 60.0 * fixedTime);

	if (GetEngineSpeedRPM() < 6500)
	{
		const Real minimumIdleTorque(5.2 * 1.3558179); //ft-lbs to Nm
		const Real onThrottleTorque(GetOutputTorque(GetEngineSpeedRPM()) * racecarController.GetThrottlePosition());
		const Real appliedEngineTorque((minimumIdleTorque < onThrottleTorque) ? onThrottleTorque : minimumIdleTorque);
		ApplyDownstreamTorque(appliedEngineTorque * revolutions, *this);
	}

	//Resistance of 1Nm for every 32 rad/s <-- THIS COMMENT MIGHT NOT BE TRUE ANYMORE...
	const Real engineResistanceTorque(GetAngularVelocity() * 0.0625);
	ApplyDownstreamTorque(-engineResistanceTorque, *this);

	//Now that all torques have been applied to the engine, step it forward in time.
	RotatingBody::Simulate();

	const Real differenceTo1000(GetEngineSpeedRPM() - 1000.0);
	if (differenceTo1000 < 0.0)
	{
		//SetAngularVelocity(Racecar::RevolutionsMinuteToDegreesSecond(1000.0f));
		const Real totalInertia(ComputeDownstreamInertia(*this));
		ApplyDownstreamTorque(-differenceTo1000 * totalInertia, *this);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::Engine::GetEngineSpeedRPM(void) const
{
	//deg   60 sec   1 rev       rev
	//--- * ---    * ---         ---
	//sec   1 min    360 deg     min


	return GetAngularVelocity() * 60.0 / 360.0;
}

//-------------------------------------------------------------------------------------------------------------------//

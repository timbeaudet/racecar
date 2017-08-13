///
/// @file
/// @details A simple base class for each of the components of the drivetrain that have rotating bodies.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "rotating_body.h"

#include "../drive_train_simulation.h"
#include "../turtle_brains/tb_math_kit.h"
#include "../turtle_brains/tb_debug_kit.h"

//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//

//Output will be inertia in kg-m^2
float Racecar::ComputeInertiaImperial(float massInPounds, float radiusInInches)
{
	const float radiusInMeters(tbMath::Convert::InchesToMeters(radiusInInches));
	return tbMath::Convert::PoundsToKilograms(massInPounds) * (radiusInMeters * radiusInMeters);
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::RevolutionsMinuteToDegreesSecond(const float revolutionsMinute)
{
	return revolutionsMinute / 60.0f * 360.0f;
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::DegreesSecondToRevolutionsMinute(const float degreesSecond)
{
	return degreesSecond * 60.0f / 360.0f;
}

//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//

Racecar::RotatingBody::RotatingBody(const float momentOfInertia) :
	mInputSource(nullptr),
	mOutputSources(),
	mInertia(momentOfInertia),
	mAngularAcceleration(0.0f),
	mAngularVelocity(0)
{
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::RotatingBody::~RotatingBody(void)
{
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::SetInputSource(RotatingBody* input)
{
	tb_error_if(nullptr != mInputSource, "Already has an input, attempting to change is illegal.");
	mInputSource = input;
}

//-------------------------------------------------------------------------------------------------------------------//

const Racecar::RotatingBody& Racecar::RotatingBody::GetExpectedInputSource(void) const
{
	tb_error_if(nullptr == mInputSource, "RotatingBody was expecting to have an input source for use.");
	return *mInputSource;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::RotatingBody& Racecar::RotatingBody::GetExpectedInputSource(void)
{
	tb_error_if(nullptr == mInputSource, "RotatingBody was expecting to have an input source for use.");
	return *mInputSource;
}

//-------------------------------------------------------------------------------------------------------------------//

bool Racecar::RotatingBody::IsOutputSource(const RotatingBody& source) const
{
	return (mOutputSources.end() != std::find(mOutputSources.begin(), mOutputSources.end(), &source));
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::AddOutputSource(RotatingBody* output)
{
	tb_error_if(mOutputSources.end() != std::find(mOutputSources.begin(), mOutputSources.end(), output), "Already connected to this output.");
	tb_error_if(nullptr == output, "Cannot connect to a null output.");
	mOutputSources.push_back(output);
}

//-------------------------------------------------------------------------------------------------------------------//

const Racecar::RotatingBody& Racecar::RotatingBody::GetExpectedOutputSource(const size_t& sourceIndex) const
{
	tb_error_if(mOutputSources.size() >= sourceIndex, "RotatingBody was expecting to have an output source for use of index: %d.", sourceIndex);
	return *mInputSource;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::RotatingBody& Racecar::RotatingBody::GetExpectedOutputSource(const size_t& sourceIndex)
{
	tb_error_if(mOutputSources.size() >= sourceIndex, "RotatingBody was expecting to have an output source for use of index: %d.", sourceIndex);
	return *mInputSource;
}

//-------------------------------------------------------------------------------------------------------------------//

const std::vector<Racecar::RotatingBody*>& Racecar::RotatingBody::GetOutputSources(void) const
{
	return mOutputSources;
}

//-------------------------------------------------------------------------------------------------------------------//

std::vector<Racecar::RotatingBody*>& Racecar::RotatingBody::GetOutputSources(void)
{
	return mOutputSources;
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::RotatingBody::ComputeDownstreamInertia(const RotatingBody& fromSource) const
{
	float downstreamInertia(GetInertia());
	for (RotatingBody* output : mOutputSources)
	{
		downstreamInertia += output->ComputeDownstreamInertia(fromSource);
	}

	return downstreamInertia;
}

//-------------------------------------------------------------------------------------------------------------------//

float Racecar::RotatingBody::ComputeUpstreamInertia(const RotatingBody& fromSource) const
{
	float upstreamInertia(GetInertia());
	if (nullptr != mInputSource)
	{
		upstreamInertia += mInputSource->ComputeUpstreamInertia(fromSource);
	}

	return upstreamInertia;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::Simulate(void)
{
	mAngularVelocity += mAngularAcceleration * DriveTrainSimulation::kFixedTime;
	SetAngularAcceleration(0.0f);

	//mPosition += mAngularVelocity * kFixedTime;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::ApplyDownstreamTorque(const float torqueNewtonMeters, const RotatingBody& fromSource)
{
	const float totalInertia(ComputeDownstreamInertia(fromSource));
	OnApplyDownstreamAcceleration(tbMath::Convert::RadiansToDegrees(torqueNewtonMeters / totalInertia), fromSource);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::ApplyUpstreamTorque(const float torqueNewtonMeters, const RotatingBody& fromSource)
{
	const float totalInertia(ComputeUpstreamInertia(fromSource));
	OnApplyUpstreamAcceleration(tbMath::Convert::RadiansToDegrees(torqueNewtonMeters / totalInertia), fromSource);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::OnApplyDownstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource)
{
	mAngularAcceleration += changeInAcceleration;
	for (RotatingBody* output : mOutputSources)
	{
		output->OnApplyDownstreamAcceleration(changeInAcceleration, fromSource);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::OnApplyUpstreamAcceleration(const float changeInAcceleration, const RotatingBody& fromSource)
{
	mAngularAcceleration += changeInAcceleration;
	if (nullptr != mInputSource)
	{
		mInputSource->OnApplyUpstreamAcceleration(changeInAcceleration, fromSource);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::SetInertia(const float inertia)
{
	mInertia = inertia;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::SetAngularAcceleration(const float angularAcceleration)
{
	mAngularAcceleration = angularAcceleration;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::SetAngularVelocity(const float angularVelocity)
{
	mAngularVelocity = angularVelocity;
}

//-------------------------------------------------------------------------------------------------------------------//

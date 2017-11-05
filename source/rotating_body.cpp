///
/// @file
/// @details A simple base class for each of the components of the drive-train that have rotating bodies.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "rotating_body.h"
#include <algorithm>

//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::kGravityConstant(10.0); //10m/s/s
static const Racecar::Real kPi(3.14159265358979323846);
static const Racecar::Real kTwoPi(kPi * 2.0);

Racecar::Real Racecar::RevolutionsMinuteToRadiansSecond(const Real& revolutionsMinute)
{
	return revolutionsMinute / 60.0 * kTwoPi;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::RadiansSecondToRevolutionsMinute(const Real& radiansSecond)
{
	return radiansSecond * 60.0 / kTwoPi;
}

//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//

Racecar::RotatingBody::RotatingBody(const Real& momentOfInertia) :
	mInputSource(nullptr),
	mOutputSources(),
	mInertia(momentOfInertia),
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
	error_if(nullptr != mInputSource, "Already has an input, attempting to change is illegal.");
	mInputSource = input;
}

//-------------------------------------------------------------------------------------------------------------------//

const Racecar::RotatingBody& Racecar::RotatingBody::GetExpectedInputSource(void) const
{
	error_if(nullptr == mInputSource, "RotatingBody was expecting to have an input source for use.");
	return *mInputSource;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::RotatingBody& Racecar::RotatingBody::GetExpectedInputSource(void)
{
	error_if(nullptr == mInputSource, "RotatingBody was expecting to have an input source for use.");
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
	error_if(mOutputSources.end() != std::find(mOutputSources.begin(), mOutputSources.end(), output), "Already connected to this output.");
	error_if(nullptr == output, "Cannot connect to a null output.");
	mOutputSources.push_back(output);
}

//-------------------------------------------------------------------------------------------------------------------//

const Racecar::RotatingBody& Racecar::RotatingBody::GetExpectedOutputSource(const size_t& sourceIndex) const
{
	error_if(sourceIndex >= mOutputSources.size(), "RotatingBody was expecting to have an output source for use of index: %d.", sourceIndex);
	return *(mOutputSources[sourceIndex]);
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::RotatingBody& Racecar::RotatingBody::GetExpectedOutputSource(const size_t& sourceIndex)
{
	error_if(sourceIndex >= mOutputSources.size(), "RotatingBody was expecting to have an output source for use of index: %d.", sourceIndex);
	return *(mOutputSources[sourceIndex]);
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

Racecar::Real Racecar::RotatingBody::ComputeDownstreamInertia(void) const
{
	Real downstreamInertia(GetInertia());
	for (RotatingBody* output : mOutputSources)
	{
		downstreamInertia += output->ComputeDownstreamInertia();
	}

	return downstreamInertia;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::RotatingBody::ComputeUpstreamInertia(void) const
{
	Real upstreamInertia(GetInertia());
	if (nullptr != mInputSource)
	{
		upstreamInertia += mInputSource->ComputeUpstreamInertia();
	}

	return upstreamInertia;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::Simulate(const Real& fixedTime)
{
	((void)fixedTime);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::ApplyDownstreamAngularImpulse(const Real& angularImpulse)
{
	const Real totalInertia(ComputeDownstreamInertia());
	OnDownstreamAngularVelocityChange(angularImpulse / totalInertia);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::ApplyUpstreamAngularImpulse(const Real& angularImpulse)
{
	const Real totalInertia(ComputeUpstreamInertia());
	OnUpstreamAngularVelocityChange(angularImpulse / totalInertia);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::OnDownstreamAngularVelocityChange(const Real& changeInAngularVelocity)
{
	mAngularVelocity += changeInAngularVelocity;
	for (RotatingBody* output : mOutputSources)
	{
		output->OnDownstreamAngularVelocityChange(changeInAngularVelocity);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::OnUpstreamAngularVelocityChange(const Real& changeInAngularVelocity)
{
	mAngularVelocity += changeInAngularVelocity;
	if (nullptr != mInputSource)
	{
		mInputSource->OnUpstreamAngularVelocityChange(changeInAngularVelocity);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::SetInertia(const Real& inertia)
{
	mInertia = inertia;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::SetAngularVelocity(const Real& angularVelocity)
{
	mAngularVelocity = angularVelocity;
}

//-------------------------------------------------------------------------------------------------------------------//

///
/// @file
/// @details A simple base class for each of the components of the drivetrain that have rotating bodies.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#include "rotating_body.h"

//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------//

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

Racecar::RotatingBody::RotatingBody(const Real momentOfInertia) :
	mInputSource(nullptr),
	mOutputSources(),
	mInertia(momentOfInertia),
	mAngularAcceleration(0.0),
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
	error_if(mOutputSources.size() >= sourceIndex, "RotatingBody was expecting to have an output source for use of index: %d.", sourceIndex);
	return *mInputSource;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::RotatingBody& Racecar::RotatingBody::GetExpectedOutputSource(const size_t& sourceIndex)
{
	error_if(mOutputSources.size() >= sourceIndex, "RotatingBody was expecting to have an output source for use of index: %d.", sourceIndex);
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

Racecar::Real Racecar::RotatingBody::ComputeDownstreamInertia(const RotatingBody& fromSource) const
{
	Real downstreamInertia(GetInertia());
	for (RotatingBody* output : mOutputSources)
	{
		downstreamInertia += output->ComputeDownstreamInertia(fromSource);
	}

	return downstreamInertia;
}

//-------------------------------------------------------------------------------------------------------------------//

Racecar::Real Racecar::RotatingBody::ComputeUpstreamInertia(const RotatingBody& fromSource) const
{
	Real upstreamInertia(GetInertia());
	if (nullptr != mInputSource)
	{
		upstreamInertia += mInputSource->ComputeUpstreamInertia(fromSource);
	}

	return upstreamInertia;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::Simulate(const Real fixedTime)
{
	mAngularVelocity += mAngularAcceleration * fixedTime;
	SetAngularAcceleration(0.0);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::ApplyDownstreamTorque(const Real torqueNewtonMeters, const RotatingBody& fromSource)
{
	const Real totalInertia(ComputeDownstreamInertia(fromSource));
	OnApplyDownstreamAcceleration(torqueNewtonMeters / totalInertia, fromSource);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::ApplyUpstreamTorque(const Real torqueNewtonMeters, const RotatingBody& fromSource)
{
	const Real totalInertia(ComputeUpstreamInertia(fromSource));
	OnApplyUpstreamAcceleration(torqueNewtonMeters / totalInertia, fromSource);
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::OnApplyDownstreamAcceleration(const Real changeInAcceleration, const RotatingBody& fromSource)
{
	mAngularAcceleration += changeInAcceleration;
	for (RotatingBody* output : mOutputSources)
	{
		output->OnApplyDownstreamAcceleration(changeInAcceleration, fromSource);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::OnApplyUpstreamAcceleration(const Real changeInAcceleration, const RotatingBody& fromSource)
{
	mAngularAcceleration += changeInAcceleration;
	if (nullptr != mInputSource)
	{
		mInputSource->OnApplyUpstreamAcceleration(changeInAcceleration, fromSource);
	}
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::SetInertia(const Real inertia)
{
	mInertia = inertia;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::SetAngularAcceleration(const Real angularAcceleration)
{
	mAngularAcceleration = angularAcceleration;
}

//-------------------------------------------------------------------------------------------------------------------//

void Racecar::RotatingBody::SetAngularVelocity(const Real angularVelocity)
{
	mAngularVelocity = angularVelocity;
}

//-------------------------------------------------------------------------------------------------------------------//

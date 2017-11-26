///
/// @file
/// @details Defines the functions that perform the basic tests for the racecar drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#include "test_kit.h"
#include <string>

const Racecar::Real Racecar::UnitTests::kTestEpsilon(0.00001);
const Racecar::Real Racecar::UnitTests::kTestFixedTimeStep(0.01); //Do not change without modifying tests, or many tests will fail.

namespace Racecar
{
	namespace UnitTests
	{
		std::string sTestMessageBuffer;
		bool sAllTestsPassed(true);
		bool sAllExpectionsPassed(true);
	}
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::ExpectedValue(Racecar::Real value, Racecar::Real expectedValue, const std::string formattedMessage, ...)
{
	va_list argumentsList;
	va_start(argumentsList, formattedMessage);
	const bool returnValue = ExpectedValueWithin(value, expectedValue, Racecar::UnitTests::kTestEpsilon, formattedMessage, argumentsList);
	va_end(argumentsList);

	return returnValue;
}

//--------------------------------------------------------------------------------------------------------------------//

bool Racecar::UnitTests::ExpectedValueWithin(Racecar::Real value, Racecar::Real expectedValue, Racecar::Real epsilon, const std::string formattedMessage, ...)
{
	if (fabs(value - expectedValue) > epsilon)
	{
		char buffer[2048];

		va_list argumentsList;
		va_start(argumentsList, formattedMessage);
		sprintf(buffer, formattedMessage.c_str(), argumentsList);
		va_end(argumentsList);

		if (false == formattedMessage.empty())
		{
			sTestMessageBuffer += " ---> ";
			sTestMessageBuffer += buffer;
			sTestMessageBuffer += "\n";
		}

		sAllExpectionsPassed = false;
		return false;
	}

	return true;
}

//--------------------------------------------------------------------------------------------------------------------//

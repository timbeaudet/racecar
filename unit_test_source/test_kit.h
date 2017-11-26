///
/// @file
/// @details Defines the functions that perform the basic tests for the racecar drive-train.
///
/// <!-- This file is made available under the terms of the MIT license(see LICENSE.md) -->
/// <!-- Copyright (c) 2017 Contributers: Tim Beaudet, -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_TestKit_h_
#define _Racecar_TestKit_h_

#include "../source/racecar.h"

#include <cstdio>
#include <string>
#define log_test(message, ...)   printf(message, ##__VA_ARGS__)

namespace Racecar
{
	namespace UnitTests
	{
		extern const Racecar::Real kTestEpsilon;
		extern const Racecar::Real kTestFixedTimeStep;
		extern bool sAllTestsPassed;
		extern bool sAllExpectionsPassed;
		extern std::string sTestMessageBuffer;

		template <typename FunctionToCall> bool PerformTest(FunctionToCall testFunction, const std::string& testName);
		bool ExpectedValue(Racecar::Real value, Racecar::Real expectedValue, const std::string formattedMessage, ...);
		bool ExpectedValueWithin(Racecar::Real value, Racecar::Real expectedValue, Racecar::Real epsilon, const std::string formattedMessage, ...);

	};
};

//--------------------------------------------------------------------------------------------------------------------//

template <typename FunctionToCall> bool Racecar::UnitTests::PerformTest(FunctionToCall testFunction, const std::string& testName)
{
	sAllExpectionsPassed = true;

	const bool testResult(testFunction() && true == sAllExpectionsPassed);
	if (true == testResult)
	{
		log_test("[  pass  ]  %s\n", testName.c_str());
	}
	else
	{
		log_test("[!-FAIL-!]  %s\n", testName.c_str());
		sAllTestsPassed = false;
	}

	if (false == sTestMessageBuffer.empty())
	{
		log_test("%s", sTestMessageBuffer.c_str());
		sTestMessageBuffer.clear();
	}

	sAllExpectionsPassed = true;
	return testResult;
}

//--------------------------------------------------------------------------------------------------------------------//

#endif /* _Racecar_TestKit_h_ */

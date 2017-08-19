///
/// @file
/// @details Defines the functions that perform the basic tests for the racecar drive-train.
///
/// <!-- Copyright (c) Tim Beaudet 2017 - All Rights Reserved -->
///-----------------------------------------------------------------------------------------------------------------///

#ifndef _Racecar_TestKit_h_
#define _Racecar_TestKit_h_

#include "../racecar/racecar.h"

#include <cstdio>

#define log_test(message, ...)   printf(message, ##__VA_ARGS__)
#define perform_test(testFunction, testName) \
	printf("%s: ", testName); \
	if (testFunction) {       \
		printf("Passed.\n");  \
	} else {                  \
		printf("Failed!\n");  \
		failedTest = true;    \
	}

namespace Racecar
{
	namespace UnitTests
	{
		extern const Racecar::Real kTestElipson;
	};
};

#endif /* _Racecar_TestKit_h_ */

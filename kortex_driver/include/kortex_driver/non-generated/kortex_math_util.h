#ifndef _KORTEX_MATH_UTIL_H_
#define _KORTEX_MATH_UTIL_H_

/*
 * Copyright (c) 2019 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed under the 
 * terms of the BSD 3-Clause license. 
 *
 * Refer to the LICENSE file for details.
 *
 */

#include <cmath>
#include "kortex_driver/Twist.h"

class KortexMathUtil
{
public:
    KortexMathUtil() {}
    ~KortexMathUtil() {}

    static double toRad(double degree);
    static double toDeg(double rad);
    static int getNumberOfTurns(double rad_not_wrapped);
    static double wrapRadiansFromMinusPiToPi(double rad_not_wrapped);
    static double wrapRadiansFromMinusPiToPi(double rad_not_wrapped, int& number_of_turns);
    static double wrapDegreesFromZeroTo360(double deg_not_wrapped);
    static double wrapDegreesFromZeroTo360(double deg_not_wrapped, int& number_of_turns);
    static double relative_position_from_absolute(double absolute_position, double min_value, double max_value);
    static double absolute_position_from_relative(double relative_position, double min_value, double max_value);
    static float findDistanceToBoundary(float value, float limit);
    
    // kortex_driver::Twist helper functions
    static kortex_driver::Twist substractTwists(const kortex_driver::Twist& a, const kortex_driver::Twist& b);
};

#endif
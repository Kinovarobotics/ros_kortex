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

class KortexMathUtil
{
public:
    KortexMathUtil() {}
    ~KortexMathUtil() {}

    static double toRad(double degree);
    static double toDeg(double rad);
    static double wrapRadiansFromMinusPiToPi(double rad_not_wrapped);
    static double wrapDegreesFromZeroTo360(double deg_not_wrapped);
};

#endif
/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include "kortex_driver/non-generated/kortex_math_util.h"

double KortexMathUtil::toRad(double degree)
{
    return degree * M_PI / 180.0;
}

double KortexMathUtil::toDeg(double rad)
{
    return rad * 180.0 / M_PI;
}

double KortexMathUtil::wrapRadiansFromMinusPiToPi(double rad_not_wrapped)
{
    bool properly_wrapped = false;
    do 
    {
        if (rad_not_wrapped > M_PI)
        {
            rad_not_wrapped -= 2.0*M_PI;
        }
        else if (rad_not_wrapped < -M_PI)
        {
            rad_not_wrapped += 2.0*M_PI;
        }
        else
        {
            properly_wrapped = true;
        }
    } while(!properly_wrapped);
    return rad_not_wrapped;
}

double KortexMathUtil::wrapDegreesFromZeroTo360(double deg_not_wrapped)
{
    bool properly_wrapped = false;
    do 
    {
        if (deg_not_wrapped > 360.0)
        {
            deg_not_wrapped -= 360.0;
        }
        else if (deg_not_wrapped < 0.0)
        {
            deg_not_wrapped += 360.0;
        }
        else
        {
            properly_wrapped = true;
        }
    } while(!properly_wrapped);
    return deg_not_wrapped;
}

double KortexMathUtil::relative_position_from_absolute(double absolute_position, double min_value, double max_value)
{
    double range = max_value - min_value;
    return (absolute_position - min_value) / range;
}

double KortexMathUtil::absolute_position_from_relative(double relative_position, double min_value, double max_value)
{
    double range = max_value - min_value;
    return relative_position * range + min_value;
}

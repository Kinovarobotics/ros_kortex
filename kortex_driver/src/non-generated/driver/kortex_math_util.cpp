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

int KortexMathUtil::getNumberOfTurns(double rad_not_wrapped)
{
    // it is between
    return 0;
}

double KortexMathUtil::wrapRadiansFromMinusPiToPi(double rad_not_wrapped)
{
    int n;
    return wrapRadiansFromMinusPiToPi(rad_not_wrapped, n);
}

double KortexMathUtil::wrapRadiansFromMinusPiToPi(double rad_not_wrapped, int& number_of_turns)
{
    bool properly_wrapped = false;
    number_of_turns = 0;
    do 
    {
        if (rad_not_wrapped > M_PI)
        {
            number_of_turns += 1;
            rad_not_wrapped -= 2.0*M_PI;
        }
        else if (rad_not_wrapped < -M_PI)
        {
            number_of_turns -= 1;
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
    int n;
    return wrapDegreesFromZeroTo360(deg_not_wrapped, n);
}

double KortexMathUtil::wrapDegreesFromZeroTo360(double deg_not_wrapped, int& number_of_turns)
{
    bool properly_wrapped = false;
    number_of_turns = 0;
    do 
    {
        if (deg_not_wrapped > 360.0)
        {
            number_of_turns += 1;
            deg_not_wrapped -= 360.0;
        }
        else if (deg_not_wrapped < 0.0)
        {
            number_of_turns -= 1;
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

float KortexMathUtil::findDistanceToBoundary(float value, float limit)
{
    float distance = std::abs(value) - limit;

    if ( distance < 0 )
    {
        distance = 0;
    }

    return distance;
}

kortex_driver::Twist KortexMathUtil::substractTwists(const kortex_driver::Twist& a, const kortex_driver::Twist& b)
{
    kortex_driver::Twist c;
    c.linear_x = a.linear_x - b.linear_x;
    c.linear_y = a.linear_y - b.linear_y;
    c.linear_z = a.linear_z - b.linear_z;
    c.angular_x = a.angular_x - b.angular_x;
    c.angular_y = a.angular_y - b.angular_y;
    c.angular_z = a.angular_z - b.angular_z;
    return c;
}
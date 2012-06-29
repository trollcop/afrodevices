/*
  June 2012

  BaseFlightPlus Rev -

  An Open Source STM32 Based Multicopter

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick

  Designed to run on Naze32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// Constrain
///////////////////////////////////////////////////////////////////////////////

float constrain(float input, float minValue, float maxValue)
{
    if (input < minValue)
        return minValue;
    else if (input > maxValue)
        return maxValue;
    else
        return input;
}

///////////////////////////////////////////////////////////////////////////////
// ITOA
///////////////////////////////////////////////////////////////////////////////

/*
** The following two functions together make up an itoa()
** implementation. Function i2a() is a 'private' function
** called by the public itoa() function.
**
** itoa() takes three arguments:
**        1) the integer to be converted,
**        2) a pointer to a character conversion buffer,
**        3) the radix for the conversion
**           which can range between 2 and 36 inclusive
**           range errors on the radix default it to base10
** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
*/

static char *i2a(unsigned i, char *a, unsigned r)
{
    if (i / r > 0)
        a = i2a(i / r, a, r);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % r];
    return a + 1;
}

char *itoa(int i, char *a, int r)
{
    if ((r < 2) || (r > 36))
        r = 10;
    if (i < 0) {
        *a = '-';
        *i2a(-(unsigned) i, a + 1, r) = 0;
    } else
        *i2a(i, a, r) = 0;

    return a;
}

///////////////////////////////////////////////////////////////////////////////
// FTOA
///////////////////////////////////////////////////////////////////////////////

char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0)                  // Rounding for x.xxx display format
        x += 0.0005f;
    else
        x -= 0.0005f;

    value = (int32_t) (x * 1000.0f);    // Convert float * 1000 to an integer

    itoa(abs(value), intString1, 10);   // Create string from abs of integer value

    if (value >= 0)
        intString2[0] = ' ';    // Positive number, add a pad space
    else
        intString2[0] = '-';    // Negative number, add a negative sign

    if (strlen(intString1) == 1) {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 2) {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 3) {
        intString2[1] = '0';
        strcat(intString2, intString1);
    } else {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    strncpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);

    return floatString;
}

///////////////////////////////////////////////////////////////////////////////
//  Standard Radian Format Limiter
////////////////////////////////////////////////////////////////////////////////

float standardRadianFormat(float angle)
{
    if (angle >= PI)
        return (angle - 2 * PI);
    else if (angle < -PI)
        return (angle + 2 * PI);
    else
        return (angle);
}

////////////////////////////////////////////////////////////////////////////////

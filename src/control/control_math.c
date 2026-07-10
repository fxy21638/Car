#include "control_math.h"

float Control_AngleDifference(float targetDeg, float actualDeg)
{
    float difference = targetDeg - actualDeg;
    while (difference > 180.0f)
        difference -= 360.0f;
    while (difference < -180.0f)
        difference += 360.0f;
    return difference;
}

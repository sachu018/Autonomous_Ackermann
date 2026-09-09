#include "kinematics.h"
#include "config.h"
#include <math.h>

void Kinematics_ComputeRPM(float V, float W, float *rpm_L, float *rpm_R)
{
    /* Clamp angular velocity so the inner wheel never reverses during a
     * forward/reverse arc — only spot turns are allowed to do that.       */
    if (V != 0.0f) {
        float w_max_safe = 2.0f * fabsf(V) / TRACK_WIDTH_M;
        if (W >  w_max_safe) W =  w_max_safe;
        if (W < -w_max_safe) W = -w_max_safe;
    }

    /* Differential drive: wheel surface velocities */
    float VL = V - (TRACK_WIDTH_M / 2.0f) * W;
    float VR = V + (TRACK_WIDTH_M / 2.0f) * W;

    /* Surface velocity → wheel RPM */
    const float circumference = 2.0f * 3.14159265f * WHEEL_RADIUS_M;
    *rpm_L = (VL / circumference) * 60.0f;
    *rpm_R = (VR / circumference) * 60.0f;

    /* Scale both wheels down proportionally if either exceeds the limit */
    float peak = fabsf(*rpm_L) > fabsf(*rpm_R) ? fabsf(*rpm_L) : fabsf(*rpm_R);
    if (peak > WHEEL_RPM_MAX) {
        float scale = WHEEL_RPM_MAX / peak;
        *rpm_L *= scale;
        *rpm_R *= scale;
    }

    /* Enforce minimum inner-wheel speed during arcs to prevent stalls.
     * Only applies when both wheels turn in the same direction.           */
    if (V != 0.0f) {
        int same_dir = ((*rpm_L > 0.0f) && (*rpm_R > 0.0f)) ||
                       ((*rpm_L < 0.0f) && (*rpm_R < 0.0f));
        if (same_dir) {
            float faster = fabsf(*rpm_L) > fabsf(*rpm_R)
                           ? fabsf(*rpm_L) : fabsf(*rpm_R);
            float slower_min = faster * MIN_RATIO;
            float sign = (V > 0.0f) ? 1.0f : -1.0f;
            if (fabsf(*rpm_L) < slower_min) *rpm_L = sign * slower_min;
            if (fabsf(*rpm_R) < slower_min) *rpm_R = sign * slower_min;
        }
    }
}

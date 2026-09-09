#ifndef KINEMATICS_H
#define KINEMATICS_H

/*
 * Differential drive kinematics.
 * Converts linear velocity V (m/s) and angular velocity W (rad/s)
 * into left and right wheel RPM.
 * Signed output: positive = forward, negative = reverse.
 */
void Kinematics_ComputeRPM(float V, float W, float *rpm_L, float *rpm_R);

#endif /* KINEMATICS_H */

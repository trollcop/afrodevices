#include "main.h"

void Mixer(s16 Throttle, s16 Roll, s16 Pitch, s16 Yaw)
{
    u8 i;

    if (Config.Mixer == QUAD_COPTER) {
        for (i = 0; i < 4; i++)
            Motors[i] = Throttle;
        // (Add) Adjust roll gyro output to motors
        Motors[MOTOR2] += Roll;
        Motors[MOTOR3] -= Roll;
        // (Add) Adjust pitch gyro output to motors
        Motors[MOTOR1] += Pitch;
        Motors[MOTOR4] -= Pitch;
        // (Add) Adjust yaw gyro output to motors
        Motors[MOTOR1] -= Yaw;
        Motors[MOTOR2] += Yaw;
        Motors[MOTOR3] += Yaw;
        Motors[MOTOR4] -= Yaw;
    } else {
        
        
    }

    // Limit lowest motor  value to avoid stopping motor
    for (i = 0; i < MAX_MOTORS; i++) {
        if (Motors[i] < MIN_THROTTLE)
            Motors[i] = MIN_THROTTLE;
    }
}

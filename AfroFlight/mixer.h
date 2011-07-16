#pragma once

// Supported mixer types. Not all are actualyl supported right now
enum MixerType { TRI_COPTER, QUAD_COPTER, QUAD_X_COPTER, Y4_COPTER, HEX_COPTER, Y6_COPTER };

void Mixer(s16 Throttle, s16 Roll, s16 Pitch, s16 Yaw);

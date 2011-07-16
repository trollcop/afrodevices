#pragma once

#define PPM_NUM_INPUTS (12)
void RC_Init(void);
s16 RC_GetChannel(u8 Channel);
s16 RC_GetDiffChannel(u8 Channel);
void RC_Calibrate(u8 MotorsEnable);
void RC_Update(void);
u8 RC_GetCommand(void);
s16 RC_Hysteresis(s16 Value);
u8 RC_GetSwitchMode(void);
u8 RC_GetQuality(void);

#define RC_THROTTLE             (0)
#define RC_PITCH                (1)
#define RC_ROLL                 (2)
#define RC_YAW                  (3)
#define RC_SWITCH               (4)

#define RC_COMMAND_NONE         (0)
#define RC_COMMAND_ARM          (1)
#define RC_COMMAND_DISARM       (2)
#define RC_COMMAND_ACCCAL       (3)
#define RC_COMMAND_GYROCAL      (4)

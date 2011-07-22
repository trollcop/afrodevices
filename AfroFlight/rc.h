#pragma once

#define PPM_NUM_INPUTS (12)
#define RC_THROTTLE_MIN (1120)

/* Initialize PPM capture system */
void RC_Init(void);
/* Check initial stick positions (and maybe calibrate later) */
void RC_Calibrate(u8 MotorsEnable);
/* Call this once per main loop() to pull control channels and update command status */
void RC_Update(void);
/* Get a value of channel directly without any changes */
s16 RC_GetChannel(u8 Channel);
/* Get last detected RC command (gyro calibration etc) */
u8 RC_GetCommand(void);
/* TODO FIX Get the flight mode based on switch (channel 5) configuration. This should be dynamic?... */
u8 RC_GetSwitchMode(void);
/* Number signifying quality of received signal - used to count received PPM frames and subtract on loss of signal */
u8 RC_GetQuality(void);

/* These are the 4 main control channels - roll/pitch/yaw/throttle. They will update in about 50Hz - so we
   can save time from recalculating stick parameters in main loop. use RC_XXX defines to access indexes */
extern s16 ControlChannels[4];

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

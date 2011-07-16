#pragma once

// The values exported by this module
/* Gyro Pitch/Roll/Yaw final measurements */
extern vs16 gyro[3];
/* Accelerometer X/Y/Z */
extern vs16 acc[3];
/* Battery voltage */
extern vs16 battery;
/* used for calibrating Gyros on ground */
extern s16 gyroZero[3];

void Sensors_Init(void);
void Gyro_Calibrate(void);
void Sensors_Read(void);
void Sensors_Debug(void);
// s16 Sensors_Get(u8 Index);
// s16 *Sensors_GetPointer(void);
void Voltage_Init(void);
u8 Voltage_Check(void);

/*
 * SMSBL.h
 * SMSBL series servo motor interface
 * Date: 2020.6.17
 * Author:
 */

#ifndef _SMSBL_H
#define _SMSBL_H

// Baud rate definitions
#define SMSBL_1M 0
#define SMSBL_0_5M 1
#define SMSBL_250K 2
#define SMSBL_128K 3
#define SMSBL_115200 4
#define SMSBL_76800 5
#define SMSBL_57600 6
#define SMSBL_38400 7

// Memory table definitions
//-------EPROM (Read-Only)--------
#define SMSBL_MODEL_L 3
#define SMSBL_MODEL_H 4

//-------EPROM (Read-Write)--------
#define SMSBL_ID 5
#define SMSBL_BAUD_RATE 6
#define SMSBL_MIN_ANGLE_LIMIT_L 9
#define SMSBL_MIN_ANGLE_LIMIT_H 10
#define SMSBL_MAX_ANGLE_LIMIT_L 11
#define SMSBL_MAX_ANGLE_LIMIT_H 12
#define SMSBL_CW_DEAD 26
#define SMSBL_CCW_DEAD 27
#define SMSBL_OFS_L 31
#define SMSBL_OFS_H 32
#define SMSBL_MODE 33

//-------SRAM (Read-Write)--------
#define SMSBL_TORQUE_ENABLE 40
#define SMSBL_ACC 41
#define SMSBL_GOAL_POSITION_L 42
#define SMSBL_GOAL_POSITION_H 43
#define SMSBL_GOAL_TIME_L 44
#define SMSBL_GOAL_TIME_H 45
#define SMSBL_GOAL_SPEED_L 46
#define SMSBL_GOAL_SPEED_H 47
#define SMSBL_LOCK 55

//-------SRAM (Read-Only)--------
#define SMSBL_PRESENT_POSITION_L 56
#define SMSBL_PRESENT_POSITION_H 57
#define SMSBL_PRESENT_SPEED_L 58
#define SMSBL_PRESENT_SPEED_H 59
#define SMSBL_PRESENT_LOAD_L 60
#define SMSBL_PRESENT_LOAD_H 61
#define SMSBL_PRESENT_VOLTAGE 62
#define SMSBL_PRESENT_TEMPERATURE 63
#define SMSBL_MOVING 66
#define SMSBL_PRESENT_CURRENT_L 69
#define SMSBL_PRESENT_CURRENT_H 70

#include "SCSerial.h"

class SMSBL : public SCSerial
{
public:
	SMSBL();
	SMSBL(u8 End);
	SMSBL(u8 End, u8 Level);
	virtual int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);					 // Standard write single servo position instruction
	virtual int RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);				 // Asynchronous write single servo position instruction (RegWriteAction takes effect)
	virtual void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]); // Synchronous write multiple servos position instruction
	virtual int WheelMode(u8 ID);														 // Constant speed mode
	virtual int WriteSpe(u8 ID, s16 Speed, u8 ACC = 0);									 // Constant speed mode control instruction
	virtual int EnableTorque(u8 ID, u8 Enable);											 // Torque control instruction
	virtual int unLockEprom(u8 ID);														 // Unlock EPROM
	virtual int LockEprom(u8 ID);														 // Lock EPROM
	virtual int CalibrationOfs(u8 ID);													 // Center position calibration
	virtual int FeedBack(int ID);														 // Feedback servo information
	virtual int ReadPos(int ID);														 // Read position
	virtual int ReadSpeed(int ID);														 // Read speed
	virtual int ReadLoad(int ID);														 // Read output voltage percentage to the motor (0~1000)
	virtual int ReadVoltage(int ID);													 // Read voltage
	virtual int ReadTemper(int ID);														 // Read temperature
	virtual int ReadMove(int ID);														 // Read moving status
	virtual int ReadCurrent(int ID);													 // Read current
private:
	u8 Mem[SMSBL_PRESENT_CURRENT_H - SMSBL_PRESENT_POSITION_L + 1];
};

#endif

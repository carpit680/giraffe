/*
 * SCS.h
 * Serial servo communication layer protocol program
 * Date: 2022.3.29
 * Author:
 */

#ifndef _SCS_H
#define _SCS_H

#include "INST.h"

class SCS
{
public:
	SCS();
	SCS(u8 End);
	SCS(u8 End, u8 Level);
	int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);				// General write command
	int regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);				// Asynchronous write command
	int RegWriteAction(u8 ID = 0xfe);								// Execute asynchronous write command
	void snycWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen); // Synchronous write command
	int writeByte(u8 ID, u8 MemAddr, u8 bDat);						// Write 1 byte
	int writeWord(u8 ID, u8 MemAddr, u16 wDat);						// Write 2 bytes
	int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);				// Read command
	int readByte(u8 ID, u8 MemAddr);								// Read 1 byte
	int readWord(u8 ID, u8 MemAddr);								// Read 2 bytes
	int Ping(u8 ID);												// Ping command
	int syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen);		// Send synchronous read packet command
	int syncReadPacketRx(u8 ID, u8 *nDat);							// Decode synchronous read return packet; returns number of bytes on success, 0 on failure
	int syncReadRxPacketToByte();									// Decode 1 byte
	int syncReadRxPacketToWrod(u8 negBit = 0);						// Decode 2 bytes; negBit indicates direction, negBit = 0 means no direction
	void syncReadBegin(u8 IDN, u8 rxLen);							// Begin synchronous read
	void syncReadEnd();												// End synchronous read
public:
	u8 Level; // Servo return level
	u8 End;	  // Processor endianness
	u8 Error; // Servo status
	u8 syncReadRxPacketIndex;
	u8 syncReadRxPacketLen;
	u8 *syncReadRxPacket;
	u8 *syncReadRxBuff;
	u16 syncReadRxBuffLen;
	u16 syncReadRxBuffMax;

protected:
	virtual int writeSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen) = 0;
	virtual int writeSCS(unsigned char bDat) = 0;
	virtual void rFlushSCS() = 0;
	virtual void wFlushSCS() = 0;

protected:
	void writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
	void Host2SCS(u8 *DataL, u8 *DataH, u16 Data); // Split a 16-bit number into 2 8-bit numbers
	u16 SCS2Host(u8 DataL, u8 DataH);			   // Combine 2 8-bit numbers into 1 16-bit number
	int Ack(u8 ID);								   // Return acknowledgment
};
#endif

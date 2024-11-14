// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com
/*
* twi.c
*
* Created: 15/03/2019 12:53:30
*  Author: IHA
*/

/* ################################################## Standard includes ################################################# */
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
/* ################################################### Project includes ################################################# */
#include "twi.h"
/* ############################################ Module Variables/Declarations ########################################### */
#define TWI_BUFFER_SIZE 10 // Max buffer size including SLA
#define TWI_WRITE_BIT 0
#define TWI_READ_BIT 1

enum {
	// General TWI Master status codes
	TWI_START = 0x08  // START has been transmitted
	,TWI_REP_START = 0x10  // Repeated START has been transmitted
	,TWI_ARB_LOST = 0x38  // Arbitration lost
	// TWI Master Transmitter status codes
	,TWI_MTX_ADR_ACK = 0x18  // SLA+W has been transmitted and ACK received
	//,TWI_MASTER_TX_ADR_WRITE_NACK = 0x20  // SLA+W has been transmitted and NACK received
	,TWI_MTX_DATA_ACK = 0x28  // Data byte has been transmitted and ACK received
	//,TWI_MASTER_TX_DATA_NACK = 0x30  // Data byte has been transmitted and NACK received
	// TWI Master Receiver status codes
	,TWI_MRX_ADR_ACK = 0x40  // SLA+R has been transmitted and ACK received
	//,TWI_MASTER_RX_ADR_READ_NACK = 0x48  // SLA+R has been transmitted and NACK received
	,TWI_MRX_DATA_ACK =  0x50  // Data byte has been received and ACK transmitted
	,TWI_MRX_DATA_NACK = 0x58  // Data byte has been received and NACK transmitted
	// TWI Slave Transmitter status codes
	,TWI_STX_ADR_ACK = 0xA8  // Own SLA+R has been received; ACK has been returned
	,TWI_STX_ADR_ACK_M_ARB_LOST = 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
	,TWI_STX_DATA_ACK = 0xB8  // Data byte in TWDR has been transmitted; ACK has been received
	,TWI_STX_DATA_NACK = 0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
	,TWI_STX_DATA_ACK_LAST_BYTE = 0xC8  // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
	// TWI Slave Receiver status codes
	,TWI_SRX_ADR_ACK = 0x60  // Own SLA+W has been received ACK has been returned
	,TWI_SRX_ADR_ACK_M_ARB_LOST = 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
	,TWI_SRX_GEN_ACK = 0x70  // General call address has been received; ACK has been returned
	,TWI_SRX_GEN_ACK_M_ARB_LOST = 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
	,TWI_SRX_ADR_DATA_ACK = 0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
	,TWI_SRX_ADR_DATA_NACK = 0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
	,TWI_SRX_GEN_DATA_ACK = 0x90  // Previously addressed with general call; data has been received; ACK has been returned
	,TWI_SRX_GEN_DATA_NACK = 0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
	,TWI_SRX_STOP_RESTART = 0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave
	// TWI Miscellaneous status codes
	,TWI_NO_RELEVANT_STATE = 0xF8  // No relevant state information available; TWINT = “0”
	//,TWI_BUS_ERROR = 0x00  // Bus error due to an illegal START or STOP condition
};

struct twiStruct { // ADT Datatype
	uint8_t slaveAddr;
	uint8_t twbrReg;  // Depends on speed
	void(*handlerCallback)(twi_p, twiReturnCode_t, uint8_t *, uint8_t);
};

typedef union // Status byte holding flags.
{
	uint8_t reg;
	struct
	{
		unsigned char lastTransOK:1;
		unsigned char unusedBits:7;
	};
} _twiStatusregister_access_t ;

static bool _twiIsInitiated = false;
static _twiStatusregister_access_t _twiStatusReg = {0};

static uint8_t _twiMessageBuffer[ TWI_BUFFER_SIZE ];    // Transceiver buffer
static uint8_t _twiMessageLen;                   // Number of bytes to be transmitted.
static uint8_t _twiCurrentState = TWI_NO_RELEVANT_STATE;      // State byte. Default set to TWI_NO_STATE.
static twi_p _twiCurrentHandler = NULL;


/* ################################################### Global Variables ################################################# */

/*-----------------------------------------------------------*/
static void _initTwiInterface(void)
{
	if (!_twiIsInitiated)
	{
		TWBR = 12;                                 // Just a dummy value
		TWDR = 0xFF;                               // Default content = SDA released.
		TWCR = (1<<TWEN)|                          // Enable TWI-interface and release TWI pins.
		(0<<TWIE)|(0<<TWINT)|                      // Disable Interrupt.
		(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests.
		(0<<TWWC);

		_twiIsInitiated = true;
	}
}

/*-----------------------------------------------------------*/
twi_p twiCreate(uint8_t slaveAddr, twiClock_t speed, uint32_t f_cpu, void(*handlerCallback)(twi_p, twiReturnCode_t, uint8_t *, uint8_t))
{
	twi_p self = malloc(sizeof(struct twiStruct));
	
	if (self == NULL)
	{
		return NULL;
	}

	self->slaveAddr = slaveAddr;

	if (speed == twi100kHz)
	{
		self->twbrReg = ((f_cpu/100000)-16)/2;
	}
	else
	{
		self->twbrReg = ((f_cpu/400000)-16)/2;
	}

	self->handlerCallback = handlerCallback;

	return self;
}

/*-----------------------------------------------------------*/
void twiDestroy(twi_p twiHandler)
{
	if (twiHandler)
	{
		free(twiHandler);
	}
}

/*-----------------------------------------------------------*/
static void _selectTwiInstance(twi_p twiHandler)
{
	if (_twiCurrentHandler != twiHandler)
	{
		_initTwiInterface();
		TWBR = twiHandler->twbrReg;
		_twiCurrentHandler = twiHandler;
	}
}

/*-----------------------------------------------------------*/
twiReturnCode_t twiTransmit(twi_p twiHandler, uint8_t * bytes, uint8_t noOfBytes )
{
	uint8_t _tmp;

	if ( NULL == twiHandler )
	{
		return TWI_NULL_HANDLER;
	}
	
	if (TWI_BUFFER_SIZE <= noOfBytes)
	{
		return TWI_MESSAGE_TO_LONG;
	}

	if (twiIsBusy())
	{
		return TWI_BUSY;
	}

	_selectTwiInstance(twiHandler);

	_twiMessageLen = noOfBytes + 1; // Number of bytes to transmit.
	_twiMessageBuffer[0] = (twiHandler->slaveAddr << 1) | TWI_WRITE_BIT;
	
	for ( _tmp = 0; _tmp < noOfBytes; _tmp++ )
	{
		_twiMessageBuffer[ _tmp + 1 ] = bytes[ _tmp ];
	}

	_twiStatusReg.reg = 0;

	_twiCurrentState = TWI_NO_RELEVANT_STATE;

	TWCR = (1<<TWEN)|                      // TWI Interface enabled.
	(1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
	(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
	(0<<TWWC);

	return TWI_TRANSMIT_OK;
}

/*-----------------------------------------------------------*/
twiReturnCode_t twiReceive(twi_p twiHandler, uint8_t noOfBytes )
{

	if ( NULL == twiHandler )
	{
		return TWI_NULL_HANDLER;
	}

	if (TWI_BUFFER_SIZE < noOfBytes)
	{
		return TWI_MESSAGE_TO_LONG;
	}

	if (twiIsBusy())
	{
		return TWI_BUSY;
	}

	_selectTwiInstance(twiHandler);

	_twiMessageLen = noOfBytes; // Number of bytes to receive.
	_twiMessageBuffer[0] = (twiHandler->slaveAddr << 1) | TWI_READ_BIT;

	_twiStatusReg.reg = 0;

	_twiCurrentState = TWI_NO_RELEVANT_STATE;

	TWCR = (1<<TWEN)|                      // TWI Interface enabled.
	(1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interrupt and clear the flag.
	(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
	(0<<TWWC);

	return TWI_RECEIVE_OK;
}

/*-----------------------------------------------------------*/
bool twiIsBusy( void )
{
	return ((TWCR & (1<<TWIE)) ? true : false); // IF TWI Interrupt is enabled then the Transceiver is busy
}

/*-----------------------------------------------------------*/
ISR(TWI_vect)
{
	static uint8_t _index;
	
	switch (TWSR)
	{
		case TWI_START:             // START has been transmitted
		case TWI_REP_START:         // Repeated START has been transmitted
		_index = 0;                  // Set buffer pointer to the TWI Address location
		case TWI_MTX_ADR_ACK:       // SLA+W has been transmitted and ACK received
		case TWI_MTX_DATA_ACK:      // Data byte has been transmitted and ACK received
		if (_index < _twiMessageLen)
		{
			TWDR = _twiMessageBuffer[_index++];
			TWCR = (1<<TWEN)|                          // TWI Interface enabled
			(1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag to send byte
			(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           //
			(0<<TWWC);                                 //
		}else                    // Send STOP after last byte
		{
			_twiStatusReg.lastTransOK = true;                 // Set status bits to completed successfully.
			TWCR = (1<<TWEN)|                                 // TWI Interface enabled
			(0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
			(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
			(0<<TWWC);                                 //

			if (NULL != _twiCurrentHandler->handlerCallback)
			{
				_twiCurrentHandler->handlerCallback(_twiCurrentHandler, TWI_TRANSMIT_OK, NULL, 0);
			}
		}
		break;

		case TWI_MRX_DATA_ACK:							// Data byte has been received and ACK transmitted
		_twiMessageBuffer[_index++] = TWDR;
		case TWI_MRX_ADR_ACK:							// SLA+R has been transmitted and ACK received
		if ( TWSR == TWI_MRX_ADR_ACK)
		{
			_index = 0;
		}
		if (_index < (_twiMessageLen-1) )              // Detect the last byte to NACK it.
		{
			TWCR = (1<<TWEN)|                          // TWI Interface enabled
			(1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag to read next byte
			(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send ACK after reception
			(0<<TWWC);                                 //
		}else											// Send NACK after next reception
		{
			TWCR = (1<<TWEN)|                          // TWI Interface enabled
			(1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag to read next byte
			(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send NACK after reception
			(0<<TWWC);                                 //
		}
		break;

		case TWI_MRX_DATA_NACK:						// Data byte has been received and NACK transmitted
		_twiMessageBuffer[_index] = TWDR;
		_twiStatusReg.lastTransOK = true;          // Set status bits to completed successfully.
		TWCR = (1<<TWEN)|                          // TWI Interface enabled
		(0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
		(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
		(0<<TWWC);                                 //

		if (NULL != _twiCurrentHandler->handlerCallback)
		{
			_twiCurrentHandler->handlerCallback(_twiCurrentHandler, TWI_RECEIVE_OK, _twiMessageBuffer, _twiMessageLen);
		}
		break;

		case TWI_ARB_LOST:          // Arbitration lost
		TWCR = (1<<TWEN)|                                 // TWI Interface enabled
		(1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interrupt and clear the flag
		(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|           // Initiate a (RE)START condition.
		(0<<TWWC);                                 //
		break;

		case TWI_MASTER_TX_ADR_WRITE_NACK:      // SLA+W has been transmitted and NACK received
		case TWI_MASTER_RX_ADR_READ_NACK:      // SLA+R has been transmitted and NACK received
		case TWI_MASTER_TX_DATA_NACK:     // Data byte has been transmitted and NACK received
		case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
		default:
		_twiCurrentState = TWSR;                                 // Store TWSR and automatically sets clears noErrors bit.
		// Reset TWI Interface
		TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
		(0<<TWIE)|(0<<TWINT)|                      // Disable Interrupt
		(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests
		(0<<TWWC);                                 //

		if (NULL != _twiCurrentHandler->handlerCallback)
		{
			_twiCurrentHandler->handlerCallback(_twiCurrentHandler, _twiCurrentState, NULL, 0);
		}
	}
}

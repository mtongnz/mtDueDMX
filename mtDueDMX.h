/*
  Copyright (c) 2018 Matthew Tong.  All right reserved.

  Credit to Michel Vergeest with his DmxDue library.  It was the starting
  point for this library. I have changed large portions of code and added
  support for 2 extra DMX ports.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3.0 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
*/

#ifndef _mtDueDMX_
#define _mtDueDMX_

// Includes Atmel CMSIS
#include <chip.h>
// Include Atmel CMSIS driver
//#include <include/usart.h>

//#include "HardwareSerial.h"


#define DMX_UNI_SIZE 512

// Use these settings to slow DMX output (doesn't work for DMX 0 as UART doesn't support it)
#define DMX_DELAY_MID_FRAME 0   // each number adds 4us to each byte
#define DMX_DELAY_END_FRAME 0   // each number adds 4us to each complete DMX frame

#define BREAKSPEED      (SystemCoreClock / 100000) / 16
#define DMXSPEED        (SystemCoreClock / 250000) / 16
#define USARTMRBASE     US_MR_USART_MODE_NORMAL | US_MR_USCLKS_MCK | US_MR_CHMODE_NORMAL | US_MR_CHRL_8_BIT
#define BREAKFORMAT   USARTMRBASE | UART_MR_PAR_SPACE   // Use parity bit to make break longer
#define DMXFORMAT     USARTMRBASE | UART_MR_PAR_MARK    // Use parity bit as first stop bit (UART doesn't support 2 stop bits)

typedef enum {
  TX_IDLE, TX_BREAK, TX_START, TX_DATA, TX_FINISH,
  RDM_BREAK, RDM_START, RDM_DATA, RDM_RX,
  RX_IDLE, RX_BREAK, RX_START, RX_DATA
} dmxState_t;

typedef enum {
  DMX_DISABLED, DMX_STOP, DMX_TX, RDM_TX, DMX_RX
} dmxMode_t;



class mtDueDMXClass {
  private:
  	Usart		    *_pUsart;
    IRQn_Type 	_dwIrq;
    uint32_t 	  _dwId;

    uint8_t     _modePin;
  	dmxState_t	_state;
    bool        _doOutput;

  	uint16_t	  _rxCnt;
  	uint16_t	  _rxLength;

  	uint16_t		_txCnt;
  	uint16_t		_txLength;

  public:
    mtDueDMXClass(Usart* pUart, IRQn_Type dwIrq, uint32_t dwId);

    void begin(uint8_t _mode, uint8_t *buffer, uint8_t modePin);
    void end(void);

    void startOutput(void);
    void stopOutput(void);

    void IrqHandler(void);
  	void setTxMaxChannels(uint16_t maxChannels);

  	uint8_t *buffer;
    uint8_t mode = DMX_DISABLED;
};

extern mtDueDMXClass mtDueDMX0;
extern mtDueDMXClass mtDueDMX1;
extern mtDueDMXClass mtDueDMX2;
extern mtDueDMXClass mtDueDMX3;
extern mtDueDMXClass mtDueDMX4;

extern mtDueDMXClass* mtDueDMX[5];
#endif

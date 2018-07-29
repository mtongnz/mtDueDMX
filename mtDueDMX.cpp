#include "mtDueDMX.h"
#include <Arduino.h>

mtDueDMXClass::mtDueDMXClass(Usart* pUsart, IRQn_Type dwIrq, uint32_t dwId)
{
  _pUsart = pUsart;
  _dwIrq = dwIrq;
  _dwId = dwId;

  for (uint16_t i = 0; i < DMX_UNI_SIZE; i++) {
    buffer[i] = 0;
  }

  setTxMaxChannels(DMX_UNI_SIZE);
}

// set max transmit channels
void mtDueDMXClass::setTxMaxChannels(uint16_t maxChannels)
{
	if (maxChannels > DMX_UNI_SIZE)
		_txLength = DMX_UNI_SIZE;
	else
		_txLength = maxChannels;
}


// initialise hardware
void mtDueDMXClass::begin(uint8_t _mode, uint8_t *pBuffer, uint8_t modePin)
{
  __disable_irq();

  mode = _mode;
  buffer = pBuffer;
  modePin = modePin;
  _doOutput = false;

  pinMode(modePin, OUTPUT);

  // Configure USART
  pmc_enable_periph_clk( _dwId );
  _pUsart->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
  _pUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
  _pUsart->US_MR = DMXFORMAT;
  _pUsart->US_BRGR = DMXSPEED;
  _pUsart->US_IDR = 0xFFFFFFFF;
  NVIC_EnableIRQ(_dwIrq);
  NVIC_SetPriority(_dwIrq, 1);

  switch (mode) {
    case DMX_RX:
      digitalWrite(modePin, LOW);
      _pUsart->US_IER = US_IER_RXRDY | US_IER_RXBRK;
      _state = RX_IDLE;
      _pUsart->US_CR = US_CR_RXEN;
      break;

    case RDM_TX:
      digitalWrite(modePin, HIGH);
      _pUsart->US_IER = US_IER_TXEMPTY;
      _state = RDM_BREAK;
      _pUsart->US_CR = US_CR_TXEN;
      break;

    default:
    case DMX_TX:
      digitalWrite(modePin, HIGH);
      _state = TX_IDLE;
      _pUsart->US_CR = US_CR_TXEN;
      break;
  }

  __enable_irq();
}

// Completely disable the DMX port
void mtDueDMXClass::end(void) {
  NVIC_DisableIRQ(_dwIrq);
  _pUsart->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
  _pUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
  _pUsart->US_IDR = 0xFFFFFFFF;
  pmc_disable_periph_clk( _dwId );
}

// Start outputting DMX
void mtDueDMXClass::startOutput(void)
{
  if (_doOutput)
    return;

  _doOutput = true;

  if (_state == TX_IDLE) {
    _state = TX_BREAK;
    _pUsart->US_IER = US_IER_TXEMPTY;
  }
}

// Stop outputting DMX - it will allow the current universe to finish outputting
void mtDueDMXClass::stopOutput(void)
{
  _doOutput = false;
}

// all usartX interupts together
void mtDueDMXClass::IrqHandler(void)
{
	uint32_t status = _pUsart->US_CSR;   // get state before data!
/*
	// Receiver Break = frame error
	if (status & US_CSR_RXBRK) {  	//check for break
		_rxState = BREAK; // break condition detected.
		_rxLength = _rxCnt; // store dmx length
		_pUsart->US_CR |= US_CR_RSTSTA;
	}

	// Did we receive data ?
	if (status & US_CSR_RXRDY) {
		uint8_t dmxByte =_pUsart->US_RHR;	// get data

		switch (_rxState) {
			case BREAK:
				// first byte is the start code
				if (dmxByte == 0) {
					_rxState = DATA;  // normal DMX start code detected
					_rxCnt = 0;       // start with channel # 1
				} else {
					_rxState = IDLE; //  wait for next BREAK !
				}
				break;
			case DATA:
				_rx_buffer[_rxCnt] = dmxByte;	// store received data into dmx data buffer
				if (_rxCnt < DMX_RX_MAX) {
					_rxCnt++; // next channel
				} else {
					_rxState = IDLE;	// wait for next break
				}
				break;
			case IDLE:
			default:
				break;
		}
	}
*/
	// tx ready
  if ((status & US_CSR_TXEMPTY) || (status & US_CSR_TXRDY)) {

  		switch (_state) {
    		case TX_BREAK:
          // Stop the second break being sent
          if (!(status & US_CSR_TXEMPTY))
            return;

          _state = TX_START;

          _pUsart->US_IDR = 0xFFFFFFFF;
          _pUsart->US_IER = US_IER_TXEMPTY;

          // Remove delay after bytes sent
          _pUsart->US_TTGR = 0;

          // Set to break mode & baud
          _pUsart->US_MR = BREAKFORMAT;
          _pUsart->US_BRGR = BREAKSPEED;


          // send 0 for break
          _pUsart->US_THR = 0;
    			break;

        case TX_START:

          _state = TX_DATA;

          // Set to DMX mode & baud
          _pUsart->US_MR = DMXFORMAT;
          _pUsart->US_BRGR = DMXSPEED;

          _pUsart->US_IDR = 0xFFFFFFFF;
          _pUsart->US_IER = US_IER_TXRDY;

          // Delay after each byte sent
          _pUsart->US_TTGR = DMX_DELAY_MID_FRAME;

          // send dmx start code
          _pUsart->US_THR = 0;
          _txCnt = 0;
          break;

    		case TX_DATA:

    			// Send byte
    			_pUsart->US_THR = buffer[_txCnt++];

          if (_txCnt == _txLength-1) {
            // Use TX empty int to ensure the delay below only happens to the final byte
            _pUsart->US_IDR = 0xFFFFFFFF;
            _pUsart->US_IER = US_IER_TXEMPTY;

          } else if (_txCnt >= _txLength) {
            if (_doOutput)
    				  _state = TX_BREAK;
            else
              _state = TX_IDLE;

            // Delay after last byte sent
            _pUsart->US_TTGR = DMX_DELAY_END_FRAME;
          }
    			break;

    		default:
        case TX_IDLE:
          // Set state to TX_IDLE so we can use startOutput function
          _state = TX_IDLE;
          _pUsart->US_IDR = 0xFFFFFFFF;
    		  break;
  		}
   	}
}



void UART_Handler( void )
{
  if (mtDueDMX0.mode != DMX_DISABLED)
    mtDueDMX0.IrqHandler();
  else
    Serial.IrqHandler();
}

void USART0_Handler( void )
{
  if (mtDueDMX1.mode != DMX_DISABLED)
    mtDueDMX1.IrqHandler() ;
  else
    Serial1.IrqHandler();
}

void USART1_Handler( void )
{
  if (mtDueDMX2.mode != DMX_DISABLED)
    mtDueDMX2.IrqHandler();
  else
    Serial2.IrqHandler();
}

void USART3_Handler( void )
{
  if (mtDueDMX3.mode != DMX_DISABLED)
    mtDueDMX3.IrqHandler();
  else
    Serial3.IrqHandler();
}

void USART2_Handler( void )
{
  if (mtDueDMX4.mode != DMX_DISABLED)
    mtDueDMX4.IrqHandler();
}

mtDueDMXClass mtDueDMX0((Usart*)UART, UART_IRQn, ID_UART);
mtDueDMXClass mtDueDMX1(USART0, USART0_IRQn, ID_USART0);
mtDueDMXClass mtDueDMX2(USART1, USART1_IRQn, ID_USART1);
mtDueDMXClass mtDueDMX3(USART3, USART3_IRQn, ID_USART3);
mtDueDMXClass mtDueDMX4(USART2, USART2_IRQn, ID_USART2);

mtDueDMXClass* mtDueDMX[5] = {&mtDueDMX0, &mtDueDMX1, &mtDueDMX2, &mtDueDMX3, &mtDueDMX4};

# mtDueDMX
Dmx Library for Arduino Due with support for 5 DMX ports.

This library is based on the core serial libraries for the due and the [DmxDue library written by Michel Vergeest](https://github.com/michelvergeest/DmxDue).

If you find this helpful and you're feeling generous, I'd love for you to buy me a beer or some gear: https://www.paypal.me/mtongnz.  Thanks heaps to all those who have donated.  It's really appreciated

## What's Working
So far, I have all 5 ports (UART and USART0-3) all working for TX only.

## Hardware
I am currently developing an Arduino Due shield with 4 fully isolated DMX ports and ethernet.  It will be available towards the end of 2018 for purchase.  It is designed specifically to work with this library but will also function on the Arduino Mega.  If you're interested in this, [let me know here](https://github.com/mtongnz).

## Feature's Coming Soon
- Obviously DMX input... Depending on my schedule, it should be done pretty soon.
- RDM output will be the final piece for this library.  It will handle discovery and TOD maintenance.  It will have a stack to add RDM commands to and will make RDM responses available to the main application.

## Edit Variant.h
Due to how the serial functions are written for the Due, we need to modify variant.h to make the interupt handlers weak.  Find each of the handler functions and put a weak declaration before them:
```
void UART_Handler(void) __attribute__((weak));
void UART_Handler(void)
{
  Serial.IrqHandler();
}

void USART0_Handler(void) __attribute__((weak));
void USART0_Handler(void)
{
  Serial1.IrqHandler();
}

void USART1_Handler(void) __attribute__((weak));
void USART1_Handler(void)
{
  Serial2.IrqHandler();
}

void USART3_Handler(void) __attribute__((weak));
void USART3_Handler(void)
{
  Serial3.IrqHandler();
}
```

## Usage
Include mtDueDMX.h and then use begin to initiate each port.  Once initiated, use startOutput() when you're ready to start DMX output.  There are 2 ways to access the ports, either an array of pointers or via the objects themselves.
```
#include <mtDueDMX.h>

// mode: must be DMX_TX for now
// buffer: a pointer to your data array
// modePin: the direction pin for the rs485 driver
mtDueDMX[0]->begin(uint8_t _mode, uint8_t *buffer, uint8_t modePin);
mtDueDMX0.begin(uint8_t _mode, uint8_t *buffer, uint8_t modePin);

// Start DMX Output
mtDueDMX[]->startOutput();

// Stop output - goes to IDLE after end of current universe
mtDueDMX[]->stopOutput();

// Immediately disables the DMX Port entirely
mtDueDMX[]->end();

// Set how many channels to output
mtDueDMX[]->setTxMaxChannels(uint16_t maxChannels);
```

## Example
Here's a quick and dirty example.
```
#include <mtDueDMX.h>

uint8_t dmxData[512];
uint8_t dmxDirPin = 7;

uint8_t dmxValue = 0;

void setup()
{
  mtDueDMX[0]->begin(DMX_TX, dmxData, dmxDirPin);
  mtDueDMX[0]->startOutput();
}

void loop()
{
  dmxData[0] = dmxValue;
  dmxData[5] = 255 - dmxValue;

  dmxValue++;
  delay(50);
}
```

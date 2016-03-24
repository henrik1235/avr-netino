/*
  I2C.h   - I2C library
  Copyright (c) 2011-2012 Wayne Truchsess.  All right reserved.
  Rev 5.0 - January 24th, 2012
          - Removed the use of interrupts completely from the library
            so TWI state changes are now polled. 
          - Added calls to lockup() function in most functions 
            to combat arbitration problems 
          - Fixed scan() procedure which left timeouts enabled 
            and set to 80msec after exiting procedure
          - Changed scan() address range back to 0 - 0x7F
          - Removed all Wire legacy functions from library
          - A big thanks to Richard Baldwin for all the testing
            and feedback with debugging bus lockups!
  Rev 4.0 - January 14th, 2012
          - Updated to make compatible with 8MHz clock frequency
  Rev 3.0 - January 9th, 2012
          - Modified library to be compatible with Arduino 1.0
          - Changed argument type from boolean to uint8_t in pullUp(), 
            setSpeed() and receiveByte() functions for 1.0 compatability
          - Modified return values for timeout feature to report
            back where in the transmission the timeout occured.
          - added function scan() to perform a bus scan to find devices
            attached to the I2C bus.  Similar to work done by Todbot
            and Nick Gammon
  Rev 2.0 - September 19th, 2011
          - Added support for timeout function to prevent 
            and recover from bus lockup (thanks to PaulS
            and CrossRoads on the Arduino forum)
          - Changed return type for stop() from void to
            uint8_t to handle timeOut function 
  Rev 1.0 - August 8th, 2011
  
  This is a modified version of the Arduino Wire/TWI 
  library.  Functions were rewritten to provide more functionality
  and also the use of Repeated Start.  Some I2C devices will not
  function correctly without the use of a Repeated Start.  The 
  initial version of this library only supports the Master.


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
Function:     I2c.begin()  - enables the I2C hardware
Parameters:   none
Return:       none

Function:     I2c.end()  - disables the I2C hardware
Parameters:   none
Return:       none

Function:     I2c.setSpeed(fast) - enables high speed 400kHz
Parameters:   (boolean)fast:  1 - high speed, 0 - low speed(default)
Return:       none

Function:     I2c.pullup(activate) - enables/disables internal pullup resistors
Parameters:   (boolean)activate: 1 - enable internal pullup resistors(default), 0 - disable internal pullup
              resistors
Return:       none

Function:     I2c.timeOut(timeOut) - Allows the user to program a time out limit to prevent
              and recover from I2C bus lockups.  I2C bus lockups have a tendency to freeze
              a program which typically requires a power cycle to restart your progrm. This
              allows the user to define a time out in which the I2C will release itself and reinitialize
              and continue on with the next function.  Setting the value to zero will disable the function.
              On a side note, be careful with setting too low a value because some devices support clock
              stretching which can increase the time before an acknowledgement is sent which could be
              misconstrued as a lockup.
              If a lock up occurs the returned parameters from Read and/or Writes will contain a 1.
Parameters:   (uint16_t)timeOut: 1 - 65535 milliseconds. (ie 1000 = 1 second time out), 0 - disabled (default)
Return:       none

Function:     2c.scan() - scans the bus for I2C devices and reports back each 7 bit address.  The timeout
              feature was implemented so if there is a problem with the bus during the scan, it will display
              on Serial Monitor that there was a problem.
Parameters:   none
Return:       none

Function:     I2c.write(address, registerAddress) - initate an I2C write operation with no data sent.
              Typically used to set the "pointer" to a register address
Parameters:   (uint8_t)address:  7 bit I2C slave address
              (uint8_t)registerAddress: Address of the register as per the datasheet
Return:       0 - successful write
              More details to come.  Returns actual error code per Atmel Datasheet

Function:     I2c.write(address, registerAddress, data) - initate an I2C write operation, single data byte.
              Typically used to send a single byte of data to a register address
Parameters:   (uint8_t)address:  7 bit I2C slave address
              (uint8_t)registerAddress: Address of the register as per the datasheet
              (uint8_t)data: Single byte of data
Return:       0 - successful write
              More details to come.  Returns actual error code per Atmel Datasheet

Function:     I2c.write(address, registerAddress, *data) - initate an I2C write operation, array of char.
              Typically used to send an array of char starting at registerAddress location.  As a side
              note there is no restriction on how many bytes may be sent unlike the Wire library
              which has a 32 byte restriction
Parameters:   (uint8_t)address:  7 bit I2C slave address
              (uint8_t)registerAddress: Address of the register as per the datasheet
              (char)*data: array of characters
Return:       0 - successful write
              More details to come.  Returns actual error code per Atmel Datasheet

Function:     I2c.write(address, registerAddress, *data, numberBytes) - initate an I2C write operation,
              array of bytes. Typically used to send an array of bytes starting at registerAddress location. 
              As a side note there is no restriction on how many bytes may be sent unlike the Wire library
              which has a 32 byte restriction
Parameters:   (uint8_t)address:  7 bit I2C slave address
              (uint8_t)registerAddress: Address of the register as per the datasheet
              (uint8_t)*data: array of bytes
              (uint8_t)numberBytes: The number of bytes in the array to be sent
Return:       0 - successful write
              More details to come.  Returns actual error code per Atmel Datasheet

Function:     I2c.read(address, numberBytes) - initiate a read operation from the current position of
              slave register pointer. The bytes will be stored in an internal buffer and will have the
              32 byte size restriction.  Data can be read out of the buffer using I2c.receive().
Parameters:   (uint8_t)address:  7 bit I2C slave address
              (uint8_t)numberBytes: The number of bytes to be read
Return:       More details to come.  Returns actual error code per Atmel Datasheet.  Unlike the Wire
              library the read operation will not return the number of bytes read, instead it will return
              the error code which can be used for debugging.

Function:     I2c.read(address, numberBytes, *dataBuffer) - initiate a read operation from the current
              position of slave register pointer. The bytes will be stored in the dataBuffer. 
              As a side note there is no restriction on how many bytes may be received unlike the Wire
              library which has a 32 byte restriction
Parameters:   (uint8_t)address:  7 bit I2C slave address
              (uint8_t)numberBytes: The number of bytes to be read
              (uint8_t)*dataBuffer: array to store read data
Return:       More details to come.  Returns actual error code per Atmel Datasheet.  Unlike the Wire
              library the read operation will not return the number of bytes read, instead it will return
              the error code which can be used for debugging.

Function:     I2c.read(address, registerAddress, numberBytes) - initiate a write operation to set the
              pointer to the registerAddress, then sending a repeated start (not a stop then start) and
              store the number of bytes in an internal buffer.  The 32 byte size restriction is imposed
              for this function.  Data can be read out of the buffer using I2c.receive().
Parameters:   (uint8_t)address:  7 bit I2C slave address
              (uint8_t)registerAddress: Starting register address to read data from
              (uint8_t)numberBytes: The number of bytes to be read
Return:       More details to come.  Returns actual error code per Atmel Datasheet.  Unlike the Wire
              library the read operation will not return the number of bytes read, instead it will return
              the error code which can be used for debugging.

Function:     I2c.read(address, registerAddress, numberBytes, *dataBuffer) -initiate a write operation
              to set the pointer to the registerAddress, then sending a repeated start (not a stop then start)
              and store the number of bytes in the dataBuffer.
              As a side note there is no restriction on how many bytes may be received unlike the Wire
              library which has a 32 byte restriction

Parameters:   (uint8_t)address:  7 bit I2C slave address
              (uint8_t)registerAddress: Starting register address to read data from
              (uint8_t)numberBytes: The number of bytes to be read
              (uint8_t)*dataBuffer: array to store read data
Return:       More details to come.  Returns actual error code per Atmel Datasheet.  Unlike the Wire
              library the read operation will not return the number of bytes read, instead it will return
              the error code which can be used for debugging.

Function:     I2c.available()  - number of unread bytes stored in the internal 32 byte buffer
Parameters:   none
Return:       number of unread bytes

Function:     I2c.receive()  - returns the first unread byte of the internal buffer.
Parameters:   none
Return:       first unread byte of the internal buffer
*/

#if(ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <inttypes.h>

#ifndef I2C_h
#define I2C_h

#define  I2C_SLOW 1   //100kHz
#define  I2C_FAST 4   //400kHz
#define  I2C_ULTRA 10 //1MHz

#define START           0x08
#define REPEATED_START  0x10
#define MT_SLA_ACK	0x18
#define MT_SLA_NACK	0x20
#define MT_DATA_ACK     0x28
#define MT_DATA_NACK    0x30
#define MR_SLA_ACK	0x40
#define MR_SLA_NACK	0x48
#define MR_DATA_ACK     0x50
#define MR_DATA_NACK    0x58
#define LOST_ARBTRTN    0x38
#define TWI_STATUS      (TWSR & 0xF8)
#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)
#define cbi(sfr, bit)   (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))

#define MAX_BUFFER_SIZE 32


class I2C
{
  public:
    I2C();
	static I2C I2c;
    void begin();
    void end();
    void timeOut(uint16_t);
    void setSpeed(uint8_t); 
    void pullup(uint8_t);
    void scan();
    uint8_t available();
    uint8_t receive();
    uint8_t write(uint8_t, uint8_t);
    uint8_t write(int, int); 
    uint8_t write(uint8_t, uint8_t, uint8_t);
	uint8_t write(uint8_t, uint8_t, uint8_t, uint8_t);
    uint8_t write(int, int, int);
    uint8_t write(uint8_t, uint8_t, char*);
    uint8_t write(uint8_t, uint8_t, uint8_t*, uint8_t);
    uint8_t read(uint8_t, uint8_t);
    uint8_t read(int, int);
    uint8_t read(uint8_t, uint8_t, uint8_t);
    uint8_t read(int, int, int);
    uint8_t read(uint8_t, uint8_t, uint8_t*);
    uint8_t read(uint8_t, uint8_t, uint8_t, uint8_t*);


  private:
    uint8_t start();
    uint8_t sendAddress(uint8_t);
    uint8_t sendByte(uint8_t);
    uint8_t receiveByte(uint8_t);
    uint8_t stop();
    void lockUp();
    uint8_t returnStatus;
    uint8_t nack;
    uint8_t data[MAX_BUFFER_SIZE];
    static uint8_t bytesAvailable;
    static uint8_t bufferIndex;
    static uint8_t totalBytes;
    static uint16_t timeOutDelay;

};

extern I2C I2c;

#endif

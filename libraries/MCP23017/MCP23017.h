/*************************************************** 
  This library is for the I2C-Portexpander MCP23017
  In this library the MCP23017:
  - Operate in Byte mode (not in Sequential mode (IOCON.SEQOP)
  - (Byte mode with IOCON.BANK = 0
     => ( address  pointer toggle between associated A/B register pairs)
		Address			Address			Access to:		Define
	IOCON.BANK = 1	IOCON.BANK = 0
		00h				00h				IODIRA			MCP_IODIRA
		10h				01h				IODIRB			MCP_IODIRB
		01h				02h				IPOLA			MCP_IPOLA
		11h				03h				IPOLB			MCP_IPOLB
		02h				04h				GPINTENA		MCP_GPINTENA
		12h				05h				GPINTENB		MCP_GPINTENB
		03h				06h				DEFVALA			MCP_DEFVALA
		13h				07h				DEFVALB			MCP_DEFVALB
		04h				08h				INTCONA			MCP_INTCONA
		14h				09h				INTCONB			MCP_INTCONB
		05h				0Ah				IOCONA			MCP_IOCONA
		15h				0Bh				IOCONA			MCP_IOCONA
		06h				0Ch				GPPUA			MCP_GPPUA
		16h				0Dh				GPPUB			MCP_GPPUB
		07h				0Eh				INTFA			MCP_INTFA
		17h				0Fh				INTFB			MCP_INTFB
		08h				10h				INTCAPA			MCP_INTCAPA
		18h				11h				INTCAPB			MCP_INTCAPB
		09h				12h				GPIOA			MCP_GPIOA
		19h				13h				GPIOB			MCP_GPIOB
		0Ah				14h				OLATA			MCP_OLATA
		1Ah				15h				OLATB			MCP_OLATB

  Author Rainer Wieland
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <inttypes.h>
#include "../I2C/I2C.h"
#ifndef MCP23017_H_
#define MCP23017_H_

// Don't forget the Wire library
class MCP23017 {
public:
  //MCP23017();
  void begin(void);
  void begin(uint8_t addr);

  uint16_t readRegister (uint8_t reg);
  uint8_t  writeRegister(uint8_t reg, uint16_t value);
  
  void setPinDirOUT(uint16_t pin);
  void setPinDirIN(uint16_t pin);
  uint16_t getPinMode();

  void setPullUp(uint16_t pin);
  void unsetPullUp(uint16_t pin);
  uint16_t getPullUp();
  uint16_t setPullUp(uint16_t pin, uint8_t setUnset);
  
  uint16_t setPolarity(uint16_t pin, uint8_t setUnset);
  uint16_t getPolarity();
  
  void digitalWrite(uint16_t pin, uint8_t setUnset);
  uint16_t digitalRead();


  uint8_t  writeGPIOS(uint16_t);
  uint16_t readGPIOS();

  void writeOLATS(uint16_t);
  uint16_t readOLATS();

 private:
  uint8_t i2cDeviceAddr;
  uint8_t ioDirReg(uint8_t pin);

};

#define MCP23017_BASEADDRESS (uint8_t)0x20
#define MCP23017_ADDRESS_MAX (uint8_t)0x07

//REGISTER ADRESSES with ICON.BANK = 0
#define	MCP_IODIRA		0x00
#define	MCP_IODIRB		0x01
#define	MCP_IPOLA		0x02
#define	MCP_IPOLB		0x03
#define	MCP_GPINTENA	0x04
#define	MCP_GPINTENB	0x05

#define	MCP_DEFVALA		0x06
#define	MCP_DEFVALB		0x07
#define	MCP_INTCONA		0x08
#define	MCP_INTCONB		0x09
#define	MCP_IOCONA		0x0A
#define	MCP_IOCONB		0x0B

#define	MCP_GPPUA		0x0C
#define	MCP_GPPUB		0x0D

#define	MCP_INTFA		0x0E
#define	MCP_INTFB		0x0F
#define	MCP_INTCAPA		0x10
#define	MCP_INTCAPB		0x11

#define	MCP_GPIOA		0x12
#define MCP_GPIOB		0x13
#define MCP_OLATA		0x14
#define	MCP_OLATB		0x15

#define MCP_ERROR		0x1B //Register not in use

#define MCP_PIN_1		0x0001
#define MCP_PIN_2		0x0002
#define MCP_PIN_3		0x0003
#define MCP_PIN_4		0x0008
#define MCP_PIN_5		0x0010
#define MCP_PIN_6		0x0020
#define MCP_PIN_7		0x0030
#define MCP_PIN_8		0x0080
#define MCP_PIN_9		0x0100
#define MCP_PIN_10		0x0200
#define MCP_PIN_11		0x0400
#define MCP_PIN_12		0x0800
#define MCP_PIN_13		0x1000
#define MCP_PIN_14		0x2000
#define MCP_PIN_15		0x4000
#define MCP_PIN_16		0x8000

#define MCP_GPA_0		0x0001
#define MCP_GPA_1		0x0002
#define MCP_GPA_2		0x0004
#define MCP_GPA_3		0x0008
#define MCP_GPA_4		0x0010
#define MCP_GPA_5		0x0020
#define MCP_GPA_6		0x0040
#define MCP_GPA_7		0x0080
#define MCP_GPB_0		0x0100
#define MCP_GPB_1		0x0200
#define MCP_GPB_2		0x0400
#define MCP_GPB_3		0x0800
#define MCP_GPB_4		0x1000
#define MCP_GPB_5		0x2000
#define MCP_GPB_6		0x4000
#define MCP_GPB_7		0x8000

#define MCP_SET			0x01
#define MCP_UNSET		0x00

extern MCP23017 mcp;
#endif

#pragma NOIV               // Do not generate interrupt vectors
//-----------------------------------------------------------------------------
//   File:      bulkloop.c
//   Contents:   Hooks required to implement USB peripheral function.
//
//   Copyright (c) 2000 Cypress Semiconductor All rights reserved
//-----------------------------------------------------------------------------
#include "fx2.h"
#include "fx2regs.h"
#include "fx2sdly.h"			// SYNCDELAY macro
extern BOOL   GotSUD;         // Received setup data flag
extern BOOL   Sleep;
extern BOOL   Rwuen;
extern BOOL   Selfpwr;

BYTE   i2cDeviceID = 0x80;
BYTE   Configuration;      // Current configuration
BYTE   AlternateSetting;   // Alternate settings

void TD_Poll(void);

// set up some bits in the special function registers
sbit FPGADIN  = IOD^7;
sbit FPGACSB  = IOD^6;
sbit SPICLOCK = IOD^3;
sbit EEDIN    = IOD^2;
sbit SPIDOUT  = IOD^1;
sbit EECSB    = IOD^0;

// allow bitwise access to an SPIDATA byte
unsigned char bdata spi;
sbit spibit7 = spi^7;
sbit spibit6 = spi^6;
sbit spibit5 = spi^5;
sbit spibit4 = spi^4;
sbit spibit3 = spi^3;
sbit spibit2 = spi^2;
sbit spibit1 = spi^1;
sbit spibit0 = spi^0;

// contents of SPIWriteByte.i
static void SPIWriteByte(unsigned char b)
{
	spi = b;

	SPIDOUT = spibit7;
	SPICLOCK = 0;
	SPICLOCK = 1;
	
	SPIDOUT = spibit6;
	SPICLOCK = 0;
	SPICLOCK = 1;
	
	SPIDOUT = spibit5;
	SPICLOCK = 0;
	SPICLOCK = 1;
	
	SPIDOUT = spibit4;
	SPICLOCK = 0;
	SPICLOCK = 1;
	
	SPIDOUT = spibit3;
	SPICLOCK = 0;
	SPICLOCK = 1;
	
	SPIDOUT = spibit2;
	SPICLOCK = 0;
	SPICLOCK = 1;
	
	SPIDOUT = spibit1;
	SPICLOCK = 0;
	SPICLOCK = 1;
	
	SPIDOUT = spibit0;
	SPICLOCK = 0;
	SPICLOCK = 1;
}

// contents of SPIReadByte.i
static void EEPROM_SPIReadByte()
{
 	SPIDOUT = 1;

	SPICLOCK = 0;
	SPICLOCK = 1;
	spibit7 = EEDIN;

	SPICLOCK = 0;
	SPICLOCK = 1;
	spibit6 = EEDIN;

	SPICLOCK = 0;
	SPICLOCK = 1;
	spibit5 = EEDIN;

	SPICLOCK = 0;
	SPICLOCK = 1;
	spibit4 = EEDIN;

	SPICLOCK = 0;
	SPICLOCK = 1;
	spibit3 = EEDIN;

	SPICLOCK = 0;
	SPICLOCK = 1;
	spibit2 = EEDIN;

	SPICLOCK = 0;
	SPICLOCK = 1;
	spibit1 = EEDIN;

	SPICLOCK = 0;
	SPICLOCK = 1;
	spibit0 = EEDIN;
}

void EEPROM_SPIWriteContinuous(unsigned char *buffer, short len)
{
	while (len-- != 0)
	{
		//spi = *buffer++;
		//#include "SPIWriteByte.i"
		SPIWriteByte(*buffer++);	
	}
}

void EEPROM_SPIWrite(unsigned char *buffer, short len)
{
	IOD = 0xFF;
	EECSB = 0;

	while (len-- != 0)
	{
		//spi = *buffer++;
		//#include "SPIWriteByte.i"
		SPIWriteByte(*buffer++);	
	}

	IOD = 0xFF;	// ensure SPI port is high
}

void EEPROM_SPIRead(unsigned char opcodeLength, unsigned char *opcode, unsigned char *buffer, short len)
{
	unsigned char i = 0;

 	IOD = 0xFF;
	EECSB = 0;

	for (i = 0; i < opcodeLength; i++)
	{
		//spi = ((opcode >> (8 * i)) & 0xFF);
		//#include "SPIWriteByte.i"
		SPIWriteByte(*opcode++);//(opcode >> (8 * i)) & 0xFF);
	}

	while (len-- != 0)
	{
		//#include "SPIReadByte.i"
		//*buffer++ = spi;
		EEPROM_SPIReadByte();
		*buffer++ = spi;
	}

   IOD = 0xFF;	// ensure SPI port is high
}

void I2C_WriteRead_BB(unsigned char *buffer, short write, short read)
{
    unsigned char *readback = buffer;
    
	// ensure that the i2c bus is in the idle state
    while (I2CS & bmSTOP) ;

	// create the i2c start condition
    I2CS = bmSTART;

	// write the device address
	I2DAT = i2cDeviceID & 0xfe;
	while(!(I2CS & bmDONE)) ;

	while (write-- != 0)
	{
		I2DAT = *buffer++;	// write data to the i2c bus
		while(!(I2CS & bmDONE)) ;
	}

	// new start condition for the readback
	I2CS = bmSTART;

	I2DAT = i2cDeviceID | 0x01;
	while(!(I2CS & bmDONE)) ;

	*readback = I2DAT;
	while(!(I2CS & bmDONE)) ;

	while (read-- != 1)
	{
	    *readback++ = I2DAT;
		while(!(I2CS & bmDONE)) ;
	}

	I2CS = bmLASTRD;
	while(!(I2CS & bmDONE)) ;

	I2CS = bmSTOP;

	*readback++ = I2DAT;
}							

void I2C_Write_BB(unsigned char *buffer, short len)
{
    // note:  buffer[0] is the device address

    // ensure that the i2c bus is in the idle state
    while (I2CS & bmSTOP) ;

    // create the i2c start condition
    I2CS = bmSTART;

	// write the device address
	I2DAT = i2cDeviceID & 0xfe;
	while(!(I2CS & bmDONE)) ;
	
	while (len-- != 0)
	{
		I2DAT = *buffer++;	// write data to the i2c bus
		while(!(I2CS & bmDONE)) ;
	}
	
	// we're done writing data, so stop the i2c
	I2CS = bmSTOP; 	
}

void I2C_Read_BB(unsigned char *buffer, short len)
{
	// note:  buffer[0] is the device address

	// ensure that the i2c bus is in the idle state
    while (I2CS & bmSTOP) ;

    // create the i2c start condition
    I2CS = bmSTART;
	I2DAT = buffer[0];
	while(!(I2CS & bmDONE)) ;
	
	while (len-- != 0)
	{
		*buffer++ = I2DAT;
		I2CS = bmLASTRD;
		while(!(I2CS & bmDONE)) ;
	}
	
	// we're done writing data, so stop the i2c
	I2CS = bmSTOP;
}

//-----------------------------------------------------------------------------
// Task Dispatcher hooks
//   The following hooks are called by the task dispatcher.
//-----------------------------------------------------------------------------
void TD_Init(void)             // Called once at startup
{
	CPUCS = ((CPUCS & ~bmCLKSPD) | bmCLKSPD1) ;
	IOA = 0x08;	// Enable to clock generator chip
	OEA = 0xFF;	// Set all port A to outputs

	SYNCDELAY;
	EP1OUTCFG = 0xB0;	// Endpoint 1 interrupt mode (for I2C communication)
	SYNCDELAY;
  	EP1INCFG = 0xB0;	// Endpoint 1 interrupt mode
  	SYNCDELAY;           
	//Shut off extra endpoints
	EP4CFG = EP6CFG = EP8CFG = 0x00; 
	SYNCDELAY;
	EP2FIFOCFG = EP4FIFOCFG = EP6FIFOCFG = EP8FIFOCFG = 0x00;
	SYNCDELAY;

	// Crank up the I2C speed
	I2CTL = 0x01;
	SYNCDELAY;

	OED = 0x02;
	IOD = 0x00;
	OEB = 0;//make sure port B is input (for SPDIF mode) 		
}

void WaitForFlash(unsigned char* buffer)
{
    buffer[1] = 0x01;

	while ((buffer[1] & 0x01) == 0x01)
	{
		buffer[0] = 0x05;
		EEPROM_SPIRead(1, &buffer[0], &buffer[1], (short)1);
    }
}

void TD_Poll(void)             // Called repeatedly while the device is idle
{
	if( !(EP1OUTCS & bmEPBUSY) )
	{
		unsigned char i;
		unsigned char opcode;
		unsigned char buffer[64] = { 0 };

		// we always receive 64 bytes, with the first being the opcode (and second usually is the length)
		opcode = EP1OUTBUF[0];
		for (i = 1; i < 64; i++) buffer[i] = EP1OUTBUF[i];
		EP1OUTBC = 0;

		if (opcode == 0x01) EEPROM_SPIWrite(&buffer[2], (short)buffer[1]);
		else if (opcode == 0x02)
		{
			//unsigned int address = (buffer[2] | (buffer[3] << 8) | (buffer[4] << 16) | (buffer[5] << 24));
			EEPROM_SPIRead(5, &buffer[2], &buffer[2], (short)buffer[1]);

			// write the data back via USB
			EP1INBUF[0] = 0x02;
			EP1INBUF[1] = buffer[1];
			for (i = 0; i < buffer[1]; i++) EP1INBUF[i + 2] = buffer[i + 2];
			EP1INBC = buffer[1] + 2;
		}
		else if (opcode == 0x03)
		{
			EEPROM_SPIRead(1, &buffer[2], &buffer[3], (short)buffer[1]);
			
			// write the data back via USB
			EP1INBUF[0] = 0x03;
			EP1INBUF[1] = buffer[1];
			for (i = 0; i < buffer[1]; i++) EP1INBUF[i + 2] = buffer[i + 3];
			EP1INBC = buffer[1] + 2;
		}
		else if (opcode == 0x04)
		{
			EEPROM_SPIRead(0, &buffer[0], &buffer[2], (short)buffer[1]);

			// write the data back via USB
			EP1INBUF[0] = 0x04;
			EP1INBUF[1] = buffer[1];
			for (i = 0; i < buffer[1]; i++) EP1INBUF[i + 2] = buffer[i + 3];
			EP1INBC = buffer[1] + 2;
		}
		else if (opcode == 0x05) 	// start a continous write
		{
			IOD = 0xFF;
			EECSB = 0;
			EEPROM_SPIWriteContinuous(&buffer[2], (short)buffer[1]);
		}
		else if (opcode == 0x06)	// continue a continuous write 
			EEPROM_SPIWriteContinuous(&buffer[2], (short)buffer[1]);
		else if (opcode == 0x07)	// end a continuous write
		{
		 	EEPROM_SPIWriteContinuous(&buffer[2], (short)buffer[1]);
			IOD = 0xFF;

			WaitForFlash(buffer);
		}
		else if (opcode == 0x08)	// continuous read start
		{
			IOD = 0xFF;
			EECSB = 0;
			EEPROM_SPIWriteContinuous(&buffer[2], (short)buffer[1]);
		}
		else if (opcode == 0x10)
		{
		 	EP1INBUF[0] = 0x00;	// version
			EP1INBUF[1] = 0x01;	// 1.
			EP1INBUF[2] = 0x01; // 0
			EP1INBC = 3;
		}
		else if (opcode == 0x30) I2C_Write_BB(&buffer[2], (short)buffer[1]);
		else if (opcode == 0x31)
		{
			I2C_Read_BB(&buffer[2], (short)buffer[1]);		    

			// write the data back via USB
			EP1INBUF[0] = 0x31;
			EP1INBUF[1] = buffer[1];
			for (i = 0; i < buffer[1]; i++) EP1INBUF[i + 2] = buffer[i + 3];
			EP1INBC = buffer[1] + 2;
		}
		else if (opcode == 0x32)
		{
			I2C_WriteRead_BB(&buffer[3], (short)buffer[1], (short)buffer[2]);

			// write the data back via USB
			EP1INBUF[0] = 0x32;
			EP1INBUF[1] = buffer[1];
			EP1INBUF[2] = buffer[2];
			for (i = 0; i < buffer[2]; i++) EP1INBUF[i + 3] = buffer[i + 3];
			EP1INBC = buffer[2] + 3;
		}
		else if (opcode == 0x33) i2cDeviceID = buffer[2];
	}
}

BOOL TD_Suspend(void)          // Called before the device goes into suspend mode
{
   return(TRUE);
}

BOOL TD_Resume(void)          // Called after the device resumes
{
   return(TRUE);
}

//-----------------------------------------------------------------------------
// Device Request hooks
//   The following hooks are called by the end point 0 device request parser.
//-----------------------------------------------------------------------------

BOOL DR_GetDescriptor(void)
{
   return(TRUE);
}

BOOL DR_SetConfiguration(void)   // Called when a Set Configuration command is received
{  

   Configuration = SETUPDAT[2];
   return(TRUE);            // Handled by user code
}

BOOL DR_GetConfiguration(void)   // Called when a Get Configuration command is received
{
   EP0BUF[0] = Configuration;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_SetInterface(void)       // Called when a Set Interface command is received
{
   AlternateSetting = SETUPDAT[2];
   return(TRUE);            // Handled by user code
}

BOOL DR_GetInterface(void)       // Called when a Set Interface command is received
{
   EP0BUF[0] = AlternateSetting;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_GetStatus(void)
{
   return(TRUE);
}

BOOL DR_ClearFeature(void)
{
   return(TRUE);
}

BOOL DR_SetFeature(void)
{
   return(TRUE);
}

BOOL DR_VendorCmnd(void)
{
   return(TRUE);
}

//-----------------------------------------------------------------------------
// USB Interrupt Handlers
//   The following functions are called by the USB interrupt jump table.
//-----------------------------------------------------------------------------

// Setup Data Available Interrupt Handler


void ISR_Sudav(void) interrupt 0
{
   
   GotSUD = TRUE;            // Set flag
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUDAV;         // Clear SUDAV IRQ
}

// Setup Token Interrupt Handler
void ISR_Sutok(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUTOK;         // Clear SUTOK IRQ
}

void ISR_Sof(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSOF;            // Clear SOF IRQ
}

void ISR_Ures(void) interrupt 0
{
   if (EZUSB_HIGHSPEED())
   {
      pConfigDscr = pHighSpeedConfigDscr;
      pOtherConfigDscr = pFullSpeedConfigDscr;
   }
   else
   {
      pConfigDscr = pFullSpeedConfigDscr;
      pOtherConfigDscr = pHighSpeedConfigDscr;
   }
   
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmURES;         // Clear URES IRQ
}

void ISR_Susp(void) interrupt 0
{
    Sleep = TRUE;
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUSP;
  
}

void ISR_Highspeed(void) interrupt 0
{
   if (EZUSB_HIGHSPEED())
   {
      pConfigDscr = pHighSpeedConfigDscr;
      pOtherConfigDscr = pFullSpeedConfigDscr;
   }
   else
   {
      pConfigDscr = pFullSpeedConfigDscr;
      pOtherConfigDscr = pHighSpeedConfigDscr;
   }

   EZUSB_IRQ_CLEAR();
   USBIRQ = bmHSGRANT;
}
void ISR_Ep0ack(void) interrupt 0
{
}
void ISR_Stub(void) interrupt 0
{
}
void ISR_Ep0in(void) interrupt 0
{
}
void ISR_Ep0out(void) interrupt 0
{
}
void ISR_Ep1in(void) interrupt 0
{
}
void ISR_Ep1out(void) interrupt 0
{
}
void ISR_Ep2inout(void) interrupt 0
{
}
void ISR_Ep4inout(void) interrupt 0
{

}
void ISR_Ep6inout(void) interrupt 0
{
}
void ISR_Ep8inout(void) interrupt 0
{
}
void ISR_Ibn(void) interrupt 0
{
}
void ISR_Ep0pingnak(void) interrupt 0
{
}
void ISR_Ep1pingnak(void) interrupt 0
{
}
void ISR_Ep2pingnak(void) interrupt 0
{
}
void ISR_Ep4pingnak(void) interrupt 0
{
}
void ISR_Ep6pingnak(void) interrupt 0
{
}
void ISR_Ep8pingnak(void) interrupt 0
{
}
void ISR_Errorlimit(void) interrupt 0
{
}
void ISR_Ep2piderror(void) interrupt 0
{
}
void ISR_Ep4piderror(void) interrupt 0
{
}
void ISR_Ep6piderror(void) interrupt 0
{
}
void ISR_Ep8piderror(void) interrupt 0
{
}
void ISR_Ep2pflag(void) interrupt 0
{
}
void ISR_Ep4pflag(void) interrupt 0
{
}
void ISR_Ep6pflag(void) interrupt 0
{
}
void ISR_Ep8pflag(void) interrupt 0
{
}
void ISR_Ep2eflag(void) interrupt 0
{
}
void ISR_Ep4eflag(void) interrupt 0
{
}
void ISR_Ep6eflag(void) interrupt 0
{
}
void ISR_Ep8eflag(void) interrupt 0
{
}
void ISR_Ep2fflag(void) interrupt 0
{
}
void ISR_Ep4fflag(void) interrupt 0
{
}
void ISR_Ep6fflag(void) interrupt 0
{
}
void ISR_Ep8fflag(void) interrupt 0
{
}
void ISR_GpifComplete(void) interrupt 0
{
}
void ISR_GpifWaveform(void) interrupt 0
{
}

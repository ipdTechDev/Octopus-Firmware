/****************************************************************************************
**  LINX - Wiring compatible device code
**
**  For more information see:           www.labviewmakerhub.com/linx
**  For support visit the forums at:    www.labviewmakerhub.com/forums/linx
**  
**  Written By Sam Kristoff
**
** BSD2 License.
****************************************************************************************/	

/****************************************************************************************
**  Defines
****************************************************************************************/		

/****************************************************************************************
**  Includes
****************************************************************************************/	
/****************************************************************************************
**  Note:
All serial 3 are commented
****************************************************************************************/	

#include "LinxDevice.h"
#include "LinxWiringDevice.h"
#include "HardwareSerial.h"
#include <iostream>
#include <stdint.h>
#include "LinxListener.h"
LinxWiringDevice linxDevice;

#if ARDUINO_VERSION >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif
 //TMC5160Stepper *driver =nullptr;

/*Not all wiring devices have these...
#ifndef EXTERNAL
	#define EXTERNAL 0
#endif
#ifndef DEFAULT
	#define DEFAULT 0 
#ifndef INTERNAL
	#define INTERNAL 2
#endif
*/
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>

/****************************************************************************************
**  Variables
****************************************************************************************/		

/****************************************************************************************
**  Constructors / Destructors 
****************************************************************************************/
LinxWiringDevice::LinxWiringDevice()
{

	//LINX API Version
	LinxApiMajor = 3;
	LinxApiMinor = 0;
	LinxApiSubminor = 0;
	//Load User Config Data From Non Volatile Storage
	userId = NonVolatileRead(NVS_USERID) << 8 | NonVolatileRead(NVS_USERID + 1);
	
}


/****************************************************************************************
**  Functions
****************************************************************************************/

void LinxWiringDevice::DelayMs(unsigned long ms)
{
	delay(ms);
} 

unsigned long LinxWiringDevice::GetMilliSeconds()
{
	return millis();
}

unsigned long LinxWiringDevice::GetSeconds()
{
	return (millis() / 1000);
}

//--------------------------------------------------------ANALOG-------------------------------------------------------

int LinxWiringDevice::AnalogRead(unsigned char numChans, unsigned char* channels, unsigned char* values)
{
	unsigned int analogValue = 0;
	unsigned char responseByteOffset = 0;
	unsigned char responseBitsRemaining = 8; 
	unsigned char dataBitsRemaining = AiResolution;
  
	values[responseByteOffset] = 0x00;    //Clear First	Response Byte   

	//Loop Over All AI channels In Command Packet
	for(int i=0; i<numChans; i++)
	{
		analogValue = analogRead(channels[i]);	
		
		dataBitsRemaining = AiResolution;

		//Byte Packet AI Values In Response Packet
		while(dataBitsRemaining > 0)
		{
			*(values+responseByteOffset) |= ( (analogValue>>(AiResolution - dataBitsRemaining)) << (8 - responseBitsRemaining) );
			//*(values+responseByteOffset) = 69;

			if(responseBitsRemaining > dataBitsRemaining)
			{
				//Current Byte Still Has Empty Bits
				responseBitsRemaining -= dataBitsRemaining;
				dataBitsRemaining = 0;
			}
			else
			{
				//Current Byte Full
				dataBitsRemaining = dataBitsRemaining - responseBitsRemaining;
				responseByteOffset++;
				responseBitsRemaining = 8;
				values[responseByteOffset] = 0x00;    //Clear Next Response Byte     
			}
		}
	}
	
	return L_OK;
}

int LinxWiringDevice::AnalogSetRef(unsigned char mode, unsigned long voltage)
{
	#if NUM_AI_INT_REFS > 0
	switch(mode)
	{
		case 0: //Default
			//analogReference(DEFAULT);
			AiRefSet = AiRefDefault;
			break;
		case 1: //Internal
			if(NumAiRefIntVals > 0)
			{
				//Check If Internal AI Ref Value Is Supported
				for(int i=0; i<NumAiRefIntVals; i++)
				{				
					//Voltage Is Supported
					if(AiRefIntVals[i] == voltage)
					{
						//analogReference(AiRefCodes[i]);
						AiRefSet = voltage;
						return L_OK;
					}
				}
				//Didn't Find Voltage
				return LANALOG_REF_VAL_ERROR;
			}
			else
			{
				//No Internal Voltages, So Internal Mode Not Supported
				return LANALOG_REF_MODE_ERROR;
			}			
			break;
		case 2: //External
			if(voltage >= AiRefExtMin && voltage <= AiRefExtMax)
			{
				//analogReference(EXTERNAL);
				AiRefSet = voltage;
				return L_OK;
			}
			else
			{
				return LANALOG_REF_VAL_ERROR;
			}
			break;
		default:
			return LANALOG_REF_MODE_ERROR;
			break;	
	}
	return L_OK;
	#endif //NUM_AI_INT_REFS > 0

	return L_FUNCTION_NOT_SUPPORTED;


}

//--------------------------------------------------------DIGITAL-------------------------------------------------------

int LinxWiringDevice::DigitalWrite(unsigned char numChans, unsigned char* channels, unsigned char* values)
{
	for(int i=0; i<numChans; i++)
	{		
		pinMode(channels[i], OUTPUT);
		digitalWrite( channels[i], (values[i/8] >> i%8) & 0x01);
	}
	
	return L_OK;
}

int LinxWiringDevice::DigitalRead(unsigned char numChans, unsigned char* channels, unsigned char* values)
{
	unsigned char bitOffset = 8;
	unsigned char byteOffset = 0;
	unsigned char retVal = 0;
 
	//Loop Over channels To Read
	for(int i=0; i<numChans; i++)
	{
		//If bitOffset Is 0 We Have To Start A New Byte, Store Old Byte And Increment OFfsets
		if(bitOffset == 0)
		{
			//Insert retVal Into Response Buffer
			values[byteOffset] = retVal;
			retVal = 0x00;
			byteOffset++;
			bitOffset = 7;      
		}
		else
		{
			bitOffset--;
		}
		
		//Read From Next Pin
		unsigned char pinNumber = channels[i];
			
		pinMode(pinNumber, INPUT);											//Set Pin As Input (Might Make This Configurable)    		
		retVal = retVal | (digitalRead(pinNumber) << bitOffset);	//Read Pin And Insert Value Into retVal
	}
	
	//Store Last Byte
	values[byteOffset] = retVal;
	
	return L_OK;
}


int LinxWiringDevice::DigitalWriteSquareWave(unsigned char channel, unsigned long freq, unsigned long duration)
{
	if(freq > 0)
	{
		pinMode(channel, OUTPUT);
		if(duration > 0)
		{
			tone(channel, freq, duration);
		}
		else
		{
			tone(channel, freq);
		}
	}
	else
	{
		noTone(channel);
	}	
	
	return L_OK;
}

int LinxWiringDevice::DigitalReadPulseWidth(unsigned char stimChan, unsigned char stimType, unsigned char respChan, unsigned char respType, unsigned long timeout, unsigned long* width)
{
	//Stimulus
	if(stimType == 1)
	{
		//High->Low->High
		pinMode(stimChan, OUTPUT);
		
		digitalWrite(stimChan, HIGH);
		delay(1);
		digitalWrite(stimChan, LOW);
		delay(1);
		digitalWrite(stimChan, HIGH);		
	}
	else if(stimType == 2)
	{
		//Low->High->Low
		pinMode(stimChan, OUTPUT);
		
		digitalWrite(stimChan, LOW);
		delay(1);
		digitalWrite(stimChan, HIGH);
		delay(1);
		digitalWrite(stimChan, LOW);		
	}
	
	//Read Pulse
	pinMode(respChan, INPUT);
	
	if(respType == 0)
	{
		*width = pulseIn(respChan, LOW, timeout);
	}
	else if(respType == 1)
	{
		*width = pulseIn(respChan, HIGH, timeout);
	}	
	
	return L_OK;	
}

//--------------------------------------------------------PWM-----------------------------------------------------------

int LinxWiringDevice::PwmSetDutyCycle(unsigned char numChans, unsigned char* channels, unsigned char* values)
{
	for(int i=0; i<numChans; i++)
	{		
		pinMode(channels[i], OUTPUT);
		analogWrite(channels[i], values[i]);
	}
	
	return L_OK;
}

//--------------------------------------------------------SPI-----------------------------------------------------------

int LinxWiringDevice::SpiOpenMaster(unsigned char channel)
{
	SPI.begin();
	return 0;
}

int LinxWiringDevice::SpiSetBitOrder(unsigned char channel, unsigned char bitOrder)
{
	SPI.setBitOrder(bitOrder);
	return 0;
}

int LinxWiringDevice::SpiSetMode(unsigned char channel, unsigned char mode)
{
	 switch(mode)
	 {
		case 0: 
			SPI.setDataMode(SPI_MODE0);
			break;
		case 1: 
			SPI.setDataMode(SPI_MODE1);
			break;
		case 2: 
			SPI.setDataMode(SPI_MODE2);
			break;
		case 3: 
			SPI.setDataMode(SPI_MODE3);
			break;
	 }
	
	return 0;
}

int LinxWiringDevice::SpiSetSpeed(unsigned char channel, unsigned long speed, unsigned long* actualSpeed)
{
	//Loop Over All Supported SPI Speeds (SPI Speeds Should Be Fastest -> Slowest)
	for(int index=0; index < NumSpiSpeeds; index++)
	{
			//If Target Speed Is greater or equal to the current supported speed use current supported speed (it's the fastest supported speed that is less or equal to the target)
			if(speed >= *(SpiSupportedSpeeds+index))
			{
				*actualSpeed = *(SpiSupportedSpeeds+index);
				SPI.setClockDivider(*(SpiSpeedCodes+index));
				break;
			}
			if(index == NumSpiSpeeds-1)
			{
				//Target speed is slower than slowest supported.  Use slowest supported
				*actualSpeed = *(SpiSupportedSpeeds+index);
				SPI.setClockDivider(*(SpiSpeedCodes+index));
			}
	}
	
	return L_OK;
}

int LinxWiringDevice::SpiWriteRead(unsigned char channel, unsigned char frameSize, unsigned char numFrames, unsigned char csChan, unsigned char csLL, unsigned char* sendBuffer, unsigned char* recBuffer)
{
	//Set CS Pin As DO
	pinMode(csChan, OUTPUT);
	
	//Set CS Pin Idle Before Starting SPI Transfer
	digitalWrite(csChan, (~csLL & 0x01) );  

	//Loop Over Frames
	for(int i=0; i<numFrames; i++)
	{
		//Start of frame, set CS Pin Active
		digitalWrite(csChan, (csLL & 0x01) );
		
		//Loop Over Bytes In Frame
		for(int j=0; j<frameSize; j++)
		{
			//Transfer Data
			unsigned char byteNum = (i*frameSize) + j;
			recBuffer[byteNum] = SPI.transfer(sendBuffer[byteNum]);
		}		
		//Frame Complete, Set CS idel
		digitalWrite(csChan, (~csLL & 0x01) );
	}	
	return 0;
}

//--------------------------------------------------------I2C-----------------------------------------------------------

//Helper To Deal With Arduino API Changes
void LinxWireWrite(unsigned char data)
{
  #if ARDUINO_VERSION < 100
    Wire.send(data);
  #else
    Wire.write(data);
  #endif 
}

int LinxWiringDevice::I2cOpenMaster(unsigned char channel)
{
	if(*(I2cRefCount+channel) > 0)
	{
		//Channel Already Open, Increment Ref Count
		*(I2cRefCount+channel) = *(I2cRefCount+channel)+1;	
	}
	else
	{
		//Channel Not Yet Open, Open And Set Refcount = 1
		 Wire.begin();			//TODO ONLY SUPPORT ONE CHANNEL ATM
	}
	return 0;
}

int LinxWiringDevice::I2cSetSpeed(unsigned char channel, unsigned long speed, unsigned long* actualSpeed)
{
	if (actualSpeed)
		*actualSpeed = 100000;  // we only support standard speed
	return L_OK;
}

int LinxWiringDevice::I2cWrite(unsigned char channel, unsigned char slaveAddress, unsigned char eofConfig, unsigned char numBytes, unsigned char* sendBuffer)
{ 
	#if ARDUINO_VERSION >= 100
		Wire.beginTransmission(slaveAddress);
		Wire.write(sendBuffer, numBytes);
		
		if(eofConfig == EOF_STOP)
		{
			Wire.endTransmission(true);
		}
		else if (eofConfig == EOF_RESTART)
		{
			Wire.endTransmission(false);
		}
		else
		{
			//EOF Not Supported, Stop Bus
			Wire.endTransmission(true);
			return LI2C_EOF;
		}	
	return L_OK;
	#else
		if(eofConfig != EOF_STOP)
		{
			//EOF Not Supported, Stop Bus
			return LI2C_EOF;
		}
		Wire.beginTransmission(slaveAddress);
		for(int i=0; i<numBytes; i++)
		{
			Wire.send(*(sendBuffer+i));
		}
		Wire.endTransmission();
		return 0;
	#endif	
}

int LinxWiringDevice::I2cRead(unsigned char channel, unsigned char slaveAddress, unsigned char eofConfig, unsigned char numBytes, unsigned int timeout, unsigned char* recBuffer)
{
	#if ARDUINO_VERSION >= 100
		if(eofConfig == EOF_STOP)
		{
			Wire.requestFrom(slaveAddress, numBytes, (uint8_t)1);
		}
		else if (eofConfig == EOF_RESTART)
		{
			Wire.requestFrom(slaveAddress, numBytes, (uint8_t)0);
		}
		else
		{
			//EOF Not Supported		
			return LI2C_EOF;
		}
	#else
		if(eofConfig != EOF_STOP)
		{
			//EOF Not Supported		
			return LI2C_EOF;
		}
		Wire.requestFrom(slaveAddress, (uint8_t)numBytes);
	#endif
		
		//Wait For Data, Potentially Timeout
		unsigned long tickCount = millis();
		while(Wire.available() < numBytes)
		{
			if( (millis() - tickCount) > timeout)
			{
				return LI2C_READ_FAIL;
			}
		}
		
		//Data Read, Read And Return
		for(int i=0; i<numBytes; i++)
		{
			#if ARDUINO_VERSION >= 100
				*(recBuffer+i) = Wire.read();
			#else
				*(recBuffer+i) = Wire.receive();
			#endif			
		}	
		return L_OK;
}

int LinxWiringDevice::I2cClose(unsigned char channel)
{
	//Function Not Supported
	return L_FUNCTION_NOT_SUPPORTED;
}
		
//--------------------------------------------------------UART----------------------------------------------------------

int LinxWiringDevice::UartOpen(unsigned char channel, unsigned long baudRate, unsigned long* actualBaud)
{
	int index = 0;
	
	for(index=0; index < NumUartSpeeds; index++)
	{
			if(baudRate < *(UartSupportedSpeeds+index))
			{		
				//Previous Index Was Closest Supported Baud Without Going Over, Index Will Be Decremented Accordingly Below.
				break;
			}			
	}
	
	//Once Loop Complets Index Is One Higher Than The Correct Baud, But Could Be Zero So Check And Decrement Accordingly
	//If The Entire Loop Runs Then index == NumUartSpeeds So Decrement It To Get Max Baud
	if(index != 0)
	{
		index = index -1;
	}
	if(channel == 0)
	{		
		#if NUM_UART_CHANS > 0
			Serial.begin(*(UartSupportedSpeeds+index));
			*actualBaud = *(UartSupportedSpeeds+index);
		#endif
	}
/*
	if(channel == 1)
	{
		#if NUM_UART_CHANS > 1
			Serial1.begin(*(UartSupportedSpeeds+index));
			*actualBaud = *(UartSupportedSpeeds+index);
		#endif
	}
/*
	if(channel == 2)
	{
		#if NUM_UART_CHANS > 2
			Serial2.begin(*(UartSupportedSpeeds+index));
			*actualBaud = *(UartSupportedSpeeds+index);
		#endif
	}
	/*
	if(channel == 3)
	{
		#if NUM_UART_CHANS > 3
			Serial3.begin(*(UartSupportedSpeeds+index));
			*actualBaud = *(UartSupportedSpeeds+index);
		#endif
	}
	*/
	return L_OK;
}

int LinxWiringDevice::UartSetBaudRate(unsigned char channel, unsigned long baudRate, unsigned long* actualBaud)
{
	//UartClose(channel);
	int retVal = UartOpen(channel, baudRate, actualBaud);
	return retVal;
	return L_OK;
}

int LinxWiringDevice::UartGetBytesAvailable(unsigned char channel, unsigned char *numBytes)
{
	if(channel == 0)
	{		
		#if NUM_UART_CHANS > 0
			*numBytes = Serial.available();
		#endif
	}
	/*
	if(channel == 1)
	{
		#if NUM_UART_CHANS > 1
			*numBytes = Serial1.available();
		#endif
	}
	/*
	if(channel == 2)
	{
		#if NUM_UART_CHANS > 2
			*numBytes = Serial2.available();
		#endif
	}
    /*
	if(channel == 3)
	{
		#if NUM_UART_CHANS > 3
			*numBytes = Serial3.available();
		#endif
	}
	*/
	return L_OK;
}

int LinxWiringDevice::UartRead(unsigned char channel, unsigned char numBytes, unsigned char* recBuffer, unsigned char* numBytesRead)
{
	#if ARDUINO_VERSION >= 100
	
		if(channel == 0)
		{	
			#if NUM_UART_CHANS > 0
				*numBytesRead = Serial.readBytes((char*)recBuffer, numBytes);
			#endif
		}
	    /*
		
		else if(channel == 1)
		{
			#if NUM_UART_CHANS > 1
				*numBytesRead = Serial1.readBytes((char*)recBuffer, numBytes);
			#endif
		}
		
        /*
		else if(channel == 2)
		{
			#if NUM_UART_CHANS > 2
				*numBytesRead = Serial2.readBytes((char*)recBuffer, numBytes);
			#endif
		}
		/*
		else if(channel == 3)
		{
			#if NUM_UART_CHANS > 3
				*numBytesRead = Serial3.readBytes((char*)recBuffer, numBytes);
			#endif
		}	
			*/	
		if(*numBytesRead !=numBytes)
		{
			return LUART_READ_FAIL;
		}
		
		return L_OK;	
	
	#else
		for(int i=0; i<numBytes; i++)
		{
			
			int data = -1;
			
			if(channel == 0)
			{	
				#if NUM_UART_CHANS > 0
					data = Serial.read();
				#endif
			}
	/*
			
			else if(channel == 1)
			{
				#if NUM_UART_CHANS > 1
					data = Serial1.read();
				#endif
			}
		/*
			else if(channel == 2)
			{
				#if NUM_UART_CHANS > 2
					data = Serial2.read();
				#endif
			}
			/*
			else if(channel == 3)
			{
				#if NUM_UART_CHANS > 3
					data = Serial3.read();
				#endif
			}			
			*/
			if(data < 0)
			{
				return LUART_READ_FAIL;
			}
			else
			{
				*(recBuffer+i) = (char)data;
			}
			
			//Read All Bytes Without Error.  Return Num Bytes Read So Listener Can Pass It To PacketizeAndSend()
			*numBytesRead = numBytes;
		}
		
		return L_OK;
	#endif
}

int LinxWiringDevice::UartWrite(unsigned char channel, unsigned char numBytes, unsigned char* sendBuffer)
{
	if(channel == 0)
	{		
		#if NUM_UART_CHANS > 0
			Serial.write(sendBuffer, numBytes);
		#endif
	}
	/*
	
	if(channel == 1)
	{
		#if NUM_UART_CHANS > 1
			Serial1.write(sendBuffer, numBytes);
		#endif
	}
	/*
	if(channel == 2)
	{
		#if NUM_UART_CHANS > 2
			Serial2.write(sendBuffer, numBytes);
		#endif
	}
	/*
	if(channel == 3)
	{
		#if NUM_UART_CHANS > 3
			Serial3.write(sendBuffer, numBytes);
		#endif
	}
	*/
	return L_OK;
}

int LinxWiringDevice::UartClose(unsigned char channel)
{
	if(channel == 0)
	{	
			

		#if NUM_UART_CHANS > 0
			Serial.end();
		#endif
	}
	/*
	 
	if(channel == 1)
	{
		#if NUM_UART_CHANS > 1
			Serial1.end();
		#endif
	}
	/*
	if(channel == 2)
	{
		#if NUM_UART_CHANS > 2
		Serial2.end();
		#endif
	}
	/*
	if(channel == 3)
	{
		#if NUM_UART_CHANS > 3
			Serial3.end();
		#endif
	}
	*/
	return L_OK;
}

//--------------------------------------------------------SERVO----------------------------------------------------------
int LinxWiringDevice::ServoOpen(unsigned char numChans, unsigned char* chans)
{
	for(int i=0; i<numChans; i++)
	{
		unsigned char pin = chans[i];
		if(Servos[pin] == 0)
		{
			//Servo Not Yet Intialized On Specified Channel, Init
			Servos[pin] = new Servo();
			Servos[pin]->attach(pin);
			
			DebugPrint("Created New Servo On Channel ");
			DebugPrintln(pin, DEC);
		}
	}
	return L_OK;
}

int LinxWiringDevice::ServoSetPulseWidth(unsigned char numChans, unsigned char* chans, unsigned short* pulseWidths)
{
	
	for(int i=0; i<numChans; i++)
	{	
		
		DebugPrint("Servo ");
		DebugPrint((unsigned long)Servos[chans[i]], DEC);
		DebugPrint(" : ");
		DebugPrintln(pulseWidths[i], DEC);
		Servos[chans[i]]->writeMicroseconds(pulseWidths[i]);		
	}
	
	return L_OK;
}


int LinxWiringDevice::ServoClose(unsigned char numChans, unsigned char* chans)
{
	for(int i=0; i<numChans; i++)
	{
		Servos[chans[i]]->detach();
		Servos[chans[i]] = 0;
	}
	return L_OK;
}
//--------------------------------------------------------GENERAL----------------------------------------------------------
void LinxWiringDevice::NonVolatileWrite(int address, unsigned char data)
{
	EEPROM.write(address, data);
}

unsigned char LinxWiringDevice::NonVolatileRead(int address)
{
	return EEPROM.read(address);
}

/* convert unsigned_char to uint16 
  unsigned char data ,** data >255**
  variable = static_cast<uint16_t>(data)
*/
 
int LinxWiringDevice::Motor_Init(uint8_t ID)
{   
	uint16_t CS;
	uint32_t EN;
	// calling a array element form the map vector didnt worked so seperated CS and EN logics 
	auto CS_map = CS_pin.find(ID);
    if ( CS_map != CS_pin.end()) {
        CS =  CS_map->second;
	}
	auto EN_map = EN_pin.find(ID);
    if ( EN_map != EN_pin.end()) {
        EN =  EN_map->second;
	}
    instances[ID]=TMC5160_INSTANCE;
	//driver_1 =  std::make_unique<TMCLik<TMC5130Stepper,1>>(CS, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
	//TMCLik<TMC5160Stepper,1>driver(CS, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
	//driver =  std::make_unique< TMC5160Stepper> (CS, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
	//driver= new TMC5160Stepper(CS, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
	SPI.begin();
 	instances[ID]->begin();
	pinMode(EN, OUTPUT); 
	digitalWrite(EN,LOW);
	return L_OK;
}
int LinxWiringDevice::Motor_config(uint8_t Motor_Id, uint32_t Velocity, uint16_t Accel,uint16_t Deccel,uint16_t RMS_current,uint8_t Hold_current, uint16_t microstep, unsigned char pullup,uint32_t Pos )
{
	uint8_t ID=Motor_Id;
 	bool res= (pullup >> 0) & 0x01;
	instances[ID]->microsteps(microstep);
 	instances[ID]->rms_current(RMS_current); 
	instances[ID]->ihold(Hold_current);
	instances[ID]-> iholddelay(1);
	instances[ID]->en_pwm_mode(true);
	instances[ID]->pwm_autoscale(true);
	instances[ID]->pwm_autograd(true);
	instances[ID]->pwm_reg(4);
	instances[ID]->pwm_freq(0b01);
	instances[ID]->tbl(1);
	instances[ID]->en_spreadCycle(true);
	instances[ID]->intpol(true);
	instances[ID]->toff(3);
 	instances[ID]->AMAX(Accel);
	instances[ID]->DMAX(Deccel);
	instances[ID]->VSTOP((Velocity/2 >= 200) ? Velocity/2 : 200); 
	instances[ID]->TZEROWAIT(1);
	instances[ID]->en_softstop(1);
	instances[ID]->a1(Accel/2);
	instances[ID]->TPWMTHRS(1000);
	instances[ID]->TCOOLTHRS(1000);
 	instances[ID]->d1(Deccel/2);
 	instances[ID]->v1(0);
	instances[ID]->XACTUAL(Pos);
	instances[ID]->diag1_pushpull(res);
	instances[ID]->diag0_int_pushpull(res);
	return L_OK;
}
int LinxWiringDevice::Move(uint8_t Motor_Id,uint32_t Velocity, unsigned char Direction)
 {
	uint8_t ID=Motor_Id;
	instances[ID]->VSTART((Velocity/2.5 >= 200) ? Velocity/2.5 : 200);
	instances[ID]->VSTOP((Velocity/2 >= 200) ? Velocity/2 : 200); 
	bool dir = (Direction >> 0) & 0x01;
	instances[ID]-> RAMPMODE(dir ? MODE_VELPOS : MODE_VELNEG);
	instances[ID]-> VMAX(Velocity);
	return L_OK;
 }
int LinxWiringDevice::Motor_stop(uint8_t Motor_Id)
 {
	pinMode(PB7,OUTPUT);
	digitalWrite(PB7,HIGH);
	uint8_t ID=Motor_Id;
	if ((instances[ID]-> RAMPMODE())>0)
		instances[ID]-> VMAX(0);
	
	else{
		//instances[ID]-> RAMPMODE(MODE_POSITION);
		//instances[ID]-> VMAX(0);
		instances[ID]-> XTARGET(0);
	}
	return L_OK;
	
 }
int LinxWiringDevice::Move_Absolute(uint8_t Motor_Id,int32_t Position,uint32_t Velocity)
 {
	uint8_t ID=Motor_Id;
	instances[ID]->VSTART((Velocity/2.5 >= 200) ? Velocity/2.5 : 200);
	instances[ID]->VSTOP((Velocity/2 >= 200) ? Velocity/2 : 200); 
	instances[ID]-> RAMPMODE(MODE_POSITION);
	instances[ID]-> VMAX(Velocity);
	instances[ID]-> XTARGET(Position);
	return L_OK;
 }
int LinxWiringDevice::Move_Relative(uint8_t Motor_Id,int32_t *steps,uint32_t Velocity)
 {
	uint8_t ID=Motor_Id;
	*steps+= instances[ID]-> XACTUAL();	
	Move_Absolute(ID,*steps,Velocity);
	//return (instances[ID]->XACTUAL());
	return L_OK;
 }
/*
int *LinxWiringDevice::Get_Data(uint8_t Motor_Id)
 {
	uint8_t ID=Motor_Id;
	static int arr[3];
	uint32_t T_Step;
	uint32_t X_Actual;
	//T_Step = instances[ID]->TSTEP();
	X_Actual =instances[ID]->XACTUAL();
	//arr[0]=T_Step;
	arr[0] =X_Actual;
	arr[1] =56000;
 	return arr;
 }
*/
uint32_t LinxWiringDevice::Get_Data(uint8_t Motor_Id)
 {
	uint8_t ID=Motor_Id;
	uint32_t X_Actual = instances[ID]->XACTUAL();
    return X_Actual;
 }
int LinxWiringDevice::Position_clear(uint8_t Motor_Id)
 {
	uint8_t ID=Motor_Id;
	instances[ID]-> RAMPMODE(MODE_HOLD);
	instances[ID]->XACTUAL(0);
	instances[ID]->pol_stop_r(1);
	return L_OK;
 }
uint8_t LinxWiringDevice::Sensor(uint8_t Pin_num,unsigned char Input, uint8_t value)
 {
	uint16_t Pin;
	auto Pin_map = Out_pin.find(Pin_num);
    if (Pin_map != Out_pin.end()) {
        Pin =  Pin_map->second;
	}
	if((Input >> 0) & 0x01){
		if (Pin_num >=0 && Pin_num <=6){
			pinMode(Pin, OUTPUT);
			digitalWrite(Pin, value==0 ? LOW : HIGH);
			}
		}
	else
		if (Pin_num >=7 && Pin_num <=14){
			pinMode(Pin, INPUT);
	    	val= digitalRead(Pin);
			
		}
	return val; 

 }
int LinxWiringDevice:: Ref_serach(uint8_t Motor_Id,uint8_t Pin_num,uint8_t vel_divider,uint32_t Home_Velocity,uint8_t pp )
 {
	uint8_t ID=Motor_Id;
	uint16_t Pin;

	if (Pin_num%2){
		Move(Motor_Id,Home_Velocity,'1');
	}
	else 
		Move(Motor_Id,Home_Velocity,'0');
	Intr(Motor_Id);
	return L_OK;

 }
 int LinxWiringDevice:: Intr(uint8_t Motor_Id)
 {
	uint16_t Pin;
	auto Pin_map = Sra_pin.find(Motor_Id);
    if (Pin_map != Sra_pin.end()) {
    Pin =  Pin_map->second;
	}
	__GPIOG_CLK_ENABLE();

  	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  	GPIO_InitStruct.Pin = Pin;
  	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;//(pp>0 ?GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING);
  	GPIO_InitStruct.Pull = GPIO_PULLUP;
  	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
	if (Motor_Id<=2){
  	HAL_NVIC_SetPriority( EXTI9_5_IRQn,10,2); 
  	HAL_NVIC_EnableIRQ( EXTI9_5_IRQn);
	}
	else {
		HAL_NVIC_SetPriority( EXTI15_10_IRQn,10,2); 
  		HAL_NVIC_EnableIRQ( EXTI15_10_IRQn);
	}
	return L_OK;
 }

 /* void EXTI9_5_IRQHandler()
 {
	delay(100);
	if (EXTI->PR&(1<<6))
	{
		EXTI->PR|=(1<<6);
	}
	//Motor_stop(0);

 }*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	pinMode(16,OUTPUT);
	digitalWrite(16,HIGH);
    //if (linxDevice != nullptr) {
	linxDevice.Motor_refStop(0);
	}
	
//}
int LinxWiringDevice::Motor_refStop(uint8_t Motor_Id)
{
	/*
	uint16_t CS;
	uint32_t EN;
	// calling a array element form the map vector didnt worked so seperated CS and EN logics 
	auto CS_map = CS_pin.find(ID);
    if ( CS_map != CS_pin.end()) {
        CS =  CS_map->second;
	}
	auto EN_map = EN_pin.find(ID);
    if ( EN_map != EN_pin.end()) {
        EN =  EN_map->second;
	}
	TMC5160Stepper driver(CS, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
	driver.begin();*/


	//instances[ID]-> VMAX(0);
    ;
	return L_OK;
}
 
/****************************************************************************************
**  LINX STM32F4 device code
**
**  For more information see:           www.labviewmakerhub.com/linx
**  For support visit the forums at:    www.labviewmakerhub.com/forums/linx
**
**  Written By Ken Sharp
**
** BSD2 License.
****************************************************************************************/

/****************************************************************************************
**  Includes
****************************************************************************************/
#include <SPI.h>
#include "utility/LinxDevice.h"
#include "utility/LinxWiringDevice.h"
#include "LinxSTM32F4.h"



/****************************************************************************************
**  Member Variables
****************************************************************************************/
//System
const unsigned char LinxSTM32F4::m_DeviceName[DEVICE_NAME_LEN] = "STM32F4";

//AI
const unsigned char LinxSTM32F4::m_AiChans[NUM_AI_CHANS] = {0, 1, 2, 3, 4, 5, 6, 7, 16, 17, 32, 33, 34, 35, 36, 37, 83, 84, 85, 86, 87, 88, 89,90};
const unsigned long LinxSTM32F4::m_AiRefIntVals[NUM_AI_INT_REFS] = {1100000};
const int LinxSTM32F4::m_AiRefCodes[NUM_AI_INT_REFS] = {INTERNAL};

//AO
//None

//DIGITAL
const unsigned char LinxSTM32F4::m_DigitalChans[NUM_DIGITAL_CHANS] = {0, 8, 16, 22,23, 30, 31, 32,33,34,35,36,38,39,45,51,52,59,60,61,62,63,64,65,66,67,68,69,70,80,81,82,89,90,91,92,93,94,95,96,97,98,99,100,101,102,105,106,107,108,109,110,111};

//PWM
const unsigned char LinxSTM32F4::m_PwmChans[NUM_PWM_CHANS] = {8,22,23,30,31};

//QE
//None

//SPI
const unsigned char LinxSTM32F4::m_SpiChans[NUM_SPI_CHANS] = {0};
unsigned long LinxSTM32F4::m_SpiSupportedSpeeds[NUM_SPI_SPEEDS] = {8000000, 4000000, 2000000, 1000000, 500000, 250000, 125000};
int LinxSTM32F4::m_SpiSpeedCodes[NUM_SPI_SPEEDS] = {SPI_CLOCK_DIV2, SPI_CLOCK_DIV4, SPI_CLOCK_DIV8, SPI_CLOCK_DIV16, SPI_CLOCK_DIV32, SPI_CLOCK_DIV64, SPI_CLOCK_DIV128};

//I2C
unsigned char LinxSTM32F4::m_I2cChans[NUM_I2C_CHANS] = {0, 1};
unsigned char LinxSTM32F4::m_I2cRefCount[NUM_I2C_CHANS];

//UART
unsigned char LinxSTM32F4::m_UartChans[NUM_UART_CHANS] = {0, 1, 2, 3};
unsigned long LinxSTM32F4::m_UartSupportedSpeeds[NUM_UART_SPEEDS] = {300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 31250, 38400, 57600, 115200};

//SERVO
Servo* LinxSTM32F4::m_Servos[NUM_SERVO_CHANS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/****************************************************************************************
**  Constructors /  Destructor
****************************************************************************************/
LinxSTM32F4::LinxSTM32F4()
{
  //DeviceFamily = 0x7;// or 0x02 
  DeviceId = 0x00;	//??
  DeviceNameLen = DEVICE_NAME_LEN;
  DeviceName =  m_DeviceName;
  ListenerBufferSize = 64;

  //LINX API Version
  LinxApiMajor = 3;
  LinxApiMinor = 0;
  LinxApiSubminor = 0;

  //DIGITAL
  NumDigitalChans = NUM_DIGITAL_CHANS;
  DigitalChans = m_DigitalChans;

  //AI
  NumAiChans = NUM_AI_CHANS;
  AiChans = m_AiChans;
  AiResolution = AI_RES_BITS;
  AiRefSet = AI_REFV;

  AiRefDefault = AI_REFV;
  AiRefSet = AI_REFV;
  AiRefCodes = m_AiRefCodes;

  NumAiRefIntVals = NUM_AI_INT_REFS;
  AiRefIntVals = m_AiRefIntVals;

  AiRefExtMin = 0;
  AiRefExtMax = 5000000;

  //AO
  NumAoChans = 0;
  AoChans = 0;

  //PWM
  NumPwmChans = NUM_PWM_CHANS;
  PwmChans = m_PwmChans;

  //QE
  NumQeChans = 0;
  QeChans = 0;


  //UART
  NumUartChans = NUM_UART_CHANS;
  UartChans = m_UartChans;
  UartMaxBaud = m_UartSupportedSpeeds[NUM_UART_SPEEDS - 1];
  NumUartSpeeds = NUM_UART_SPEEDS;
  UartSupportedSpeeds = m_UartSupportedSpeeds;

  //I2C
  NumI2cChans = NUM_I2C_CHANS;
  I2cChans = m_I2cChans;
  I2cRefCount = m_I2cRefCount;

  //SPI
  NumSpiChans = NUM_SPI_CHANS;
  SpiChans = m_SpiChans;
  NumSpiSpeeds = NUM_SPI_SPEEDS;
  SpiSupportedSpeeds = m_SpiSupportedSpeeds;
  SpiSpeedCodes = m_SpiSpeedCodes;

  //CAN
  NumCanChans = 0;
  CanChans = 0;

  //SERVO
  NumServoChans = NUM_SERVO_CHANS;
  //ServoChans = m_ServoChans;
  ServoChans = m_DigitalChans;
	Servos = m_Servos;

  //If Debuging Is Enabled Call EnableDebug()
#if DEBUG_ENABLED >= 0
  EnableDebug(DEBUG_ENABLED);
#endif
}
/*
int LinxSTM32F4::AnalogSetRef(unsigned char mode, unsigned long voltage)
{
  // There are no user-accessible analog references on the STM32F4
  return L_FUNCTION_NOT_SUPPORTED;
}
*/
//Destructor
LinxSTM32F4::~LinxSTM32F4()
{
  //Handle Any Device Clean Up Here.
	//UartClose();
}

/****************************************************************************************
**  Functions
****************************************************************************************/

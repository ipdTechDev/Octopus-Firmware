/****************************************************************************************
**  LINX header for STM32F
**
**  For more information see:           www.labviewmakerhub.com/linx
**  For support visit the forums at:    www.labviewmakerhub.com/forums/linx
**
**  Written By Ken Sharp
**
** BSD2 License.
****************************************************************************************/

#ifndef LINX_ARDUINO_UNO_H
#define LINX_ARDUINO_UNO_H

/****************************************************************************************
**  Defines
****************************************************************************************/
#define DEVICE_NAME_LEN 8

#define NUM_AI_CHANS 24
#define AI_RES_BITS 12
#define AI_REFV 3300000
#define NUM_AI_INT_REFS 1

#define NUM_DIGITAL_CHANS 53

#define NUM_PWM_CHANS 6

#define NUM_SPI_CHANS 1
#define NUM_SPI_SPEEDS 7

#define NUM_I2C_CHANS 2

#define NUM_UART_CHANS 4
#define NUM_UART_SPEEDS 13

#define NUM_SERVO_CHANS NUM_DIGITAL_CHANS

/****************************************************************************************
**  Includes
****************************************************************************************/

#include "utility/LinxDevice.h"
#include "utility/LinxWiringDevice.h"
#include "utility/LinxArduino.h"
//#include <LinxESP32WifiListener.h>


class LinxSTM32F4 : public  LinxArduino//LinxWiringDevice
{
  public:
    /****************************************************************************************
    **  Variables
    ****************************************************************************************/
    //System
    static const unsigned char m_DeviceName[DEVICE_NAME_LEN];

    //AI
    static const unsigned char m_AiChans[NUM_AI_CHANS];
    static const unsigned long m_AiRefIntVals[NUM_AI_INT_REFS];
    static const int m_AiRefCodes[NUM_AI_INT_REFS];

    //AO
    //None

    //DIGITAL
    static const unsigned char m_DigitalChans[NUM_DIGITAL_CHANS];

    //PWM
    static const unsigned char m_PwmChans[NUM_PWM_CHANS];

    //SPI
    static const unsigned char m_SpiChans[NUM_SPI_CHANS];
    static unsigned long m_SpiSupportedSpeeds[NUM_SPI_SPEEDS];
    static int m_SpiSpeedCodes[NUM_SPI_SPEEDS];

    //I2C
    static unsigned char m_I2cChans[NUM_I2C_CHANS];
    static unsigned char m_I2cRefCount[NUM_I2C_CHANS];

    //UART
    static unsigned char m_UartChans[NUM_UART_CHANS];
    static unsigned long m_UartSupportedSpeeds[NUM_UART_SPEEDS];

    //Servo
    static const unsigned char m_ServoChans[NUM_SERVO_CHANS];
		static Servo* m_Servos[NUM_SERVO_CHANS];  
    /****************************************************************************************
    **  Constructors /  Destructor
    ****************************************************************************************/
    LinxSTM32F4();

    ~LinxSTM32F4();

    /****************************************************************************************
    **  Functions
    ****************************************************************************************/
    //int AnalogSetRef(unsigned char mode, unsigned long voltage);

  private:
    /****************************************************************************************
    **  Variables
    ****************************************************************************************/


    /****************************************************************************************
    **  Functions
    ****************************************************************************************/


};


#endif //LINX_ARDUINO_UNO_H

/****************************************************************************************
**  LINX header for Wiring compatible devices.
**
**  For more information see:           www.labviewmakerhub.com/linx
**  For support visit the forums at:    www.labviewmakerhub.com/forums/linx
**
**  Written By Sam Kristoff
**
** BSD2 License.
****************************************************************************************/

#ifndef LINX_WIRING_H
#define LINX_WIRING_H
#ifndef EXTERNAL
	#define EXTERNAL 0
#endif
#define R_SENSE 0.075f//0.11f

#define SW_MOSI          PA7 
#define SW_MISO          PA6 
#define SW_SCK           PA5 
/*
#ifndef DEFAULT
	#define DEFAULT 0
#endif
*/
#ifndef INTERNAL
	#define INTERNAL 2
#endif
#define MODE_POSITION  0
#define MODE_VELPOS    1
#define MODE_VELNEG    2
#define MODE_HOLD      3
#define __HAL_GPIO_EXTI_GET_IT(__EXTI_LINE__) (EXTI->PR & (__EXTI_LINE__))
#define __HAL_GPIO_EXTI_CLEAR_IT(__EXTI_LINE__) (EXTI->PR = (__EXTI_LINE__))
/****************************************************************************************
**  Includes
****************************************************************************************/
#include "LinxDevice.h"
#include <Servo.h>
#include <TMCStepper.h>
#include<map>
#include<memory>
#define TMC5160_INSTANCE std::unique_ptr<TMCLik<TMC5160Stepper,1>>(new TMCLik<TMC5160Stepper,1>(CS, R_SENSE, SW_MOSI, SW_MISO, SW_SCK))


/****************************************************************************************
**  Variables
****************************************************************************************/

class LinxWiringDevice : public LinxDevice
{
  public:
    template<class TMC, int Motor_ID>
    class TMCLik : public TMC{
     public :
      TMCLik(const uint16_t CS, const float RS, const uint16_t pinMOSI, const uint16_t pinMISO, const uint16_t pinSCK) :
      TMC(CS, RS, pinMOSI, pinMISO, pinSCK)//,tmc(std:make_unique<TMC>(CS, RS, pinMOSI, pinMISO, pinSCK))
      {}
      //TMCLik(std:: unique_ptr<TMC>tmc_ptr):TMC(*tmc_ptr),tmc(std::move(tmc_ptr)){}
      //std:: unique_ptr<TMC>tmc;
     
    };
 
    /****************************************************************************************
    **  Variables
    ****************************************************************************************/
    unsigned char NumAiRefIntVals;					//Number Of Internal AI Reference Voltages
    const unsigned long* AiRefIntVals;				//Supported AI Reference Voltages (uV)
    const int* AiRefCodes;								//AI Ref Values (AI Ref Macros In Wiring Case)

    unsigned long AiRefExtMin;							//Min External AI Ref Value (uV)
    unsigned long AiRefExtMax;					    //Min External AI Ref Value (uV)

    unsigned char NumUartSpeeds;					//Number Of Support UART Buads
    unsigned long* UartSupportedSpeeds;			//Supported UART Bauds Frequencies

    unsigned char NumSpiSpeeds;					//Number Of Supported SPI Speeds
    unsigned long* SpiSupportedSpeeds;			//Supported SPI Clock Frequencies
    int* SpiSpeedCodes;									//SPI Speed Values (Clock Divider Macros In Wiring Case)

    unsigned char* I2cRefCount;						//Number Opens - Closes On I2C Channel

    Servo** Servos;	
    /****************************************************************************************
    **  Constructors
    ****************************************************************************************/
    LinxWiringDevice();
   
    /****************************************************************************************
    **  Functions
    ****************************************************************************************/

    //Analog
    virtual int AnalogRead(unsigned char numChans, unsigned char* channels, unsigned char* values);
    virtual int AnalogSetRef(unsigned char mode, unsigned long voltage);

    //DIGITAL
    virtual int DigitalWrite(unsigned char numChans, unsigned char* channels, unsigned char* values);
    virtual int DigitalRead(unsigned char numChans, unsigned char* channels, unsigned char* values);
    virtual int DigitalWriteSquareWave(unsigned char channel, unsigned long freq, unsigned long duration);
    virtual int DigitalReadPulseWidth(unsigned char stimChan, unsigned char stimType, unsigned char respChan, unsigned char respType, unsigned long timeout, unsigned long* width);

    //PWM
    virtual int PwmSetDutyCycle(unsigned char numChans, unsigned char* channels, unsigned char* values);

    //SPI
    virtual int SpiOpenMaster(unsigned char channel);
    virtual int SpiSetBitOrder(unsigned char channel, unsigned char bitOrder);
    virtual int SpiSetMode(unsigned char channel, unsigned char mode);
    virtual int SpiSetSpeed(unsigned char channel, unsigned long speed, unsigned long* actualSpeed);
    virtual int SpiWriteRead(unsigned char channel, unsigned char frameSize, unsigned char numFrames, unsigned char csChan, unsigned char csLL, unsigned char* sendBuffer, unsigned char* recBuffer);

    //I2C
    virtual int I2cOpenMaster(unsigned char channel);
    virtual int I2cSetSpeed(unsigned char channel, unsigned long speed, unsigned long* actualSpeed);
    virtual int I2cWrite(unsigned char channel, unsigned char slaveAddress, unsigned char eofConfig, unsigned char numBytes, unsigned char* sendBuffer);
    virtual int I2cRead(unsigned char channel, unsigned char slaveAddress, unsigned char eofConfig, unsigned char numBytes, unsigned int timeout, unsigned char* recBuffer);
    virtual int I2cClose(unsigned char channel);

    //UART
    virtual int UartOpen(unsigned char channel, unsigned long baudRate, unsigned long* actualBaud);
    virtual int UartSetBaudRate(unsigned char channel, unsigned long baudRate, unsigned long* actualBaud);
    virtual int UartGetBytesAvailable(unsigned char channel, unsigned char *numBytes);
    virtual int UartRead(unsigned char channel, unsigned char numBytes, unsigned char* recBuffer, unsigned char* numBytesRead);
    virtual int UartWrite(unsigned char channel, unsigned char numBytes, unsigned char* sendBuffer);
    virtual int UartClose(unsigned char channel);

    //Servo
    virtual int ServoOpen(unsigned char numChans, unsigned char* chans);
    virtual int ServoSetPulseWidth(unsigned char numChans, unsigned char* chans, unsigned short* pulseWidths);
    virtual int ServoClose(unsigned char numChans, unsigned char* chans);

    //General -
    virtual unsigned long GetMilliSeconds();
    virtual unsigned long GetSeconds();
    virtual void DelayMs(unsigned long ms);
    virtual void NonVolatileWrite(int address, unsigned char data);
    virtual unsigned char NonVolatileRead(int address);
    //Debug

    //TMC5160 Stepper 
    virtual int Motor_Init( uint8_t ID);
    virtual int Motor_config(uint8_t Motor_Id,uint32_t Velocity ,uint16_t Accel,uint16_t Deccel,uint16_t RMS_current,uint8_t Hold_current, uint16_t microstep, unsigned char pullup,uint32_t Pos=0 );
    virtual int Move(uint8_t Motor_Id,uint32_t Velocity, unsigned char Direction);
    virtual int Motor_stop(uint8_t Motor_Id);
    virtual int Move_Absolute(uint8_t Motor_Id,int32_t Position,uint32_t Velocity);
    virtual int Move_Relative(uint8_t Motor_Id,int32_t *steps,uint32_t Velocity);
    //virtual int  *Get_Data(uint8_t Motor_Id);
    virtual uint32_t  Get_Data(uint8_t Motor_Id);
    virtual int Position_clear(uint8_t Motor_Id);
    virtual uint8_t Sensor(uint8_t Pin_num,unsigned char Input,uint8_t value);
    virtual int Ref_serach(uint8_t Motor_Id,uint8_t Pin_num,uint8_t vel_divider,uint32_t Home_Velocity,uint8_t pp);
    virtual int Intr(uint8_t ID);
    virtual int Motor_refStop(uint8_t Motor_Id);
    //virtual void EXTI9_5_IRQHandler(void);

 
    
     protected:
    /****************************************************************************************
    **  Variables
    ****************************************************************************************/
    
    /****************************************************************************************
    **  Functions
    ****************************************************************************************/

  private:
    uint8_t ID;
    GPIO_InitTypeDef GPIO_InitStruct;
    std::map<uint8_t,std::unique_ptr<TMCLik<TMC5160Stepper,1>>>instances;
     //std:: unique_ptr<TMC5160Stepper> driver; //TMC5160Stepper*driver 
     //std:: unique_ptr<TMCLik<TMC5130Stepper,1>>driver;
     //TMC5160Stepper*driver;
    const std::map<uint8_t, uint16_t> CS_pin = {
      {0, 36},
      {1, 59},
	    {2, 38},
      {3, 39},
      {4, 82},
      {5, 68},
	    {6, 65},
      {7, 51}	
	    };
    const std::map<uint8_t, uint16_t> EN_pin = {
      {0, 94},
      {1, 95},
	    {2, 101},
      {3, 0},
      {4, 98},
      {5, 81},
	    {6, 52},
      {7, 64}	
	   };
     const std::map<uint8_t, uint16_t> Out_pin={
      {0, 8},
      {1, 69},
	    {2, 61},
      {3, 62},
      {4, 63},
      {5, 16},
      {6, 23},
      {7, 102},
	    {8, 105},
      {9, 106},
      {10, 107},
      {11, 108},
	    {12, 109},
      {13, 110},
      {14, 111}   
     };
     const std::map<uint8_t, uint16_t> Sra_pin={
      {0, GPIO_PIN_6},
	    {1, GPIO_PIN_9},
      {2, GPIO_PIN_10},
      {3, GPIO_PIN_11},
      {4, GPIO_PIN_12},
	    {5, GPIO_PIN_13},
      {6, GPIO_PIN_14},
      {7, GPIO_PIN_15},
     };

     	uint8_t val =0;
    /****************************************************************************************
    **  Functions
    ****************************************************************************************/
    void LinxWireWrite();
    

};

#endif //LINX_WIRING_H

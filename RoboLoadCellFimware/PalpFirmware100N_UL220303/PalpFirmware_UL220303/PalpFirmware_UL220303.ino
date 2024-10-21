#include <ADC.h>
#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;
//#include <MsTimer2.h>
//#include <FlexiTimer2.h>
//#include <TimerThree.h>
#include <Wire.h>
//IntervalTimer imuTimer, printTimer;

PeriodicTimer eventTimer(GPT1), printTimer(GPT2), imuTimer(TCK); // generate a timer from the pool (Pool: 2xGPT, 16xTMR(QUAD), 20xTCK)
//PeriodicTimer eventTimer(GPT1), printTimer(GPT2); // generate a timer from the pool (Pool: 2xGPT, 16xTMR(QUAD), 20xTCK)

#include <BasicLinearAlgebra.h>// library for Geometry and Transformations
using namespace BLA;

//IMU ICM20948 setup variables and initializations

#include "Arduino-ICM20948.h"

ArduinoICM20948 icm20948;
ArduinoICM20948Settings icmSettings =
{
  .i2c_speed = 400000,                // i2c clock speed
  .is_SPI = false,                    // Enable SPI, if disable use i2c
  .cs_pin = 10,                       // SPI chip select pin
  .spi_speed = 7000000,               // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                          // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = true,           // Enables gyroscope output
  .enable_accelerometer = true,       // Enables accelerometer output
  .enable_magnetometer = true,        // Enables magnetometer output // Enables quaternion output
  .enable_gravity = true,             // Enables gravity vector output
  .enable_linearAcceleration = true,  // Enables linear acceleration output
  .enable_quaternion6 = true,         // Enables quaternion 6DOF output
  .enable_quaternion9 = true,         // Enables quaternion 9DOF output
  .enable_har = true,                 // Enables activity recognition
  .enable_steps = true,               // Enables step counter
  .gyroscope_frequency = 225,           // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 225,       // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 70,        // Max frequency = 70, min frequency = 1 
  .gravity_frequency = 225,             // Max frequency = 225, min frequency = 1
  .linearAcceleration_frequency = 225,  // Max frequency = 225, min frequency = 1
  .quaternion6_frequency = 225,        // Max frequency = 225, min frequency = 50
  .quaternion9_frequency = 225,        // Max frequency = 225, min frequency = 50
  .har_frequency = 225,                // Max frequency = 225, min frequency = 50
  .steps_frequency = 225               // Max frequency = 225, min frequency = 50
  
};

const uint8_t number_i2c_addr = 2;
uint8_t poss_addresses[number_i2c_addr] = {0X69, 0X68};
uint8_t ICM_address;
bool ICM_found = false;


void i2c_scan(){
    uint8_t error;
    for(uint8_t add_int = 0; add_int < number_i2c_addr; add_int++ ){
        Serial.printf("Scanning 0x%02X for slave...", poss_addresses[add_int]);
        Wire.beginTransmission(poss_addresses[add_int]);
        error = Wire.endTransmission();
        if (error == 0){
            Serial.println("found.");
            if(poss_addresses[add_int] == 0x69 || poss_addresses[add_int] == 0x68){
                Serial.println("\t- address is ICM.");
                ICM_address = poss_addresses[add_int];
                ICM_found = true;
            }
        }
    }
}

//#include <MadgwickAHRS.h>
//Madgwick madgwickAHRS;//Includes sebastian madgwick Filter.... see MadgwickAHRS.cpp

//#include <MahonyAHRS.h>//Includes Robert Mahony Nonlinear complementary Filter.... see MahonyAHRS.cpp
//Mahony mahonyAHRS;

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

//-----------SD Card library and Variable Declarations-----------------------------
#include "SdFat.h"

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 0
/*
  Change the value of SD_CS_PIN if you are using SPI and
  your hardware does not use the default value, SS.  
  Common values are:
  Arduino Ethernet shield: pin 4
  Sparkfun SD shield: pin 8
  Adafruit SD shields and modules: pin 10
*/

/*// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
#endif  // HAS_SDIO_CLASS */

const int8_t DISABLE_CS_PIN = -1;

const uint8_t SD_CS_PIN = 10;

#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50))

// Try to select the best SD_FAT_TYPE.
#if SD_FAT_TYPE == 0
SdFat sd;
File file;
File root;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
File32 root;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
ExFile root;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
FsFile root;
#endif  // SD_FAT_TYPE

cid_t m_cid;

uint32_t SrNo;
char line[300];
// Store error strings in flash to save RAM.
#define error(s) sd.errorHalt(&Serial, F(s))

//-------------------------Variable Declarations-------------------------------

ADC *adc = new ADC(); // adc object
uint16_t elapsMicro;
volatile static unsigned long startMillis=0, nowMillis=0;
volatile static int16_t buttonCount =0,lastButtonVal=0,buttonVal=0,resetFlag=0;
volatile double elapsTime; 
static uint8_t calibFlag = 1, arrCnt=0; uint16_t calibCount = 4000;
volatile unsigned long loopCnt=0;


double Voff[3],FVolts[3];
volatile uint8_t printInterval_millis = 10; //10 millis seconds
uint16_t readForceInterval = 1000;// 1000 microseconds
uint8_t sampleSize = 25;
const int bufferLength = 25;// sample Size and buffer length should be same
volatile static unsigned long previousMillis=0;
unsigned long previousMicros=0;
volatile static unsigned long currentMillis=0;
unsigned long currentMicros=0;

double avgVx1,avgVy1,avgVz1;
double MaxVLx = -5.0, MaxVLy = -5.0, MaxVLz = -5.0,MinVLx = 5.0, MinVLy = 5.0, MinVLz = 5.0;
//double MaxVRx = -5.0, MaxVRy = -5.0, MaxVRz = -5.0,MinVRx = 5.0, MinVRy = 5.0, MinVRz = 5.0;
double MedianVLx,MedianVLy,MedianVLz;
double RangeVLx,RangeVLy,RangeVLz;
double offsetFLx,offsetFLy,offsetFLz;

//Define RGB LED Pins
//Common Anode RGB LED
const int redPin = 5;
const int greenPin = 6;
const int bluePin = 7;
const int defaultLedPin = 13;

//Define Button Interrupt pin
const int buttonPin = 4;

//Define Force Sensor Pins 
// Q1 Force Sensor Pin Config
const int FrcX1Pin = A8; // ADC0 Fx1
const int FrcY1Pin = A7; // ADC0 Fy1
const int FrcZ1Pin = A6; // ADC0 Fz1

// Force Sensor (Left) Calibration Matrix - UL220309 (Q1-vk2022a)
const double FCL11 = 25.2312, FCL12 = -0.6627,  FCL13 = -0.4853;
const double FCL21 =  0.1246, FCL22 = 24.3898, FCL23 = 0.6799;
const double FCL31 = 0.4528, FCL32 = 0.3138,  FCL33 = 50.9386;

/*// Force Sensor (Left) Calibration Matrix - UL220303 (Q1-vk2022b)
const double FCL11 = 25.6015, FCL12 = -1.2078,  FCL13 = -0.7097;
const double FCL21 =  0.3287, FCL22 = 24.2049, FCL23 = 1.7050;
const double FCL31 = -0.5876, FCL32 = 0.5815,  FCL33 = 50.1527;*/

// VoltageReferecePin
//const int VoltRefPin = A20; //ADC1 Vref(manual)

//int ValX1 = 0; int ValY1 = 0; int ValZ1 = 0;

double Vx1Val[bufferLength]; double Vy1Val[bufferLength]; double Vz1Val[bufferLength]; 
//double Vx2Val[bufferLength]; double Vy2Val[bufferLength]; double Vz2Val[bufferLength];
double FrcXLVal[bufferLength]; double FrcYLVal[bufferLength]; double FrcZLVal[bufferLength];
//double FrcXRVal[bufferLength];double FrcYRVal[bufferLength];double FrcZRVal[bufferLength];

/*double Vx1Val[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double Vy1Val[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double Vz1Val[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

double Vx2Val[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double Vy2Val[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double Vz2Val[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

double FrcXLVal[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double FrcYLVal[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double FrcZLVal[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

double FrcXRVal[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double FrcYRVal[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double FrcZRVal[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};*/
String inStr;
char QSTMString[512];
char CalibString[512];
volatile double printString[18],imuParam[12];

double *frcVolts, *Volts2Frc; 

double vZL, vXL, vYL;
double vXLavg,vYLavg,vZLavg;
double FHLx, FHLy, FHLz, FHLrms;
float wt_volt_err = 0.00;
//FHRx, FHRy, FHRz, FHRrms, F3Dx,F3Dy,F3Dz,F3Drms;

float Ax,Ay,Az;
float Gx,Gy,Gz,Mx,My,Mz;
float accRMS, gyroRMS;
float Axg,Ayg,Azg;

double *eulerAng;

// Quaternions variable diclaration
volatile float QnW, QnX, QnY, QnZ;//Madgwick filter Quat Variables
volatile float Yaw, Pitch, Roll, deltaT;
float quat_w, quat_x, quat_y, quat_z;// ICM20948 quat variables
float icm_Yaw, icm_Pitch, icm_Roll, *arrMat;

void setup() {
    Wire.begin();
    // TWBR = 12;  // 400 kbit/sec I2C speed
    //madgwickAHRS.begin(500.0f);// Madgwick update at every 500 Hz = 2 milliseconds
    //mahonyAHRS.begin(500.0f);//Mahony update at every 500 Hz = per 2 milliseconds
    //Timer3.initialize(2000);// IMU filter initialization as Timer3
    //Timer3.attachInterrupt(updateIMU_filter); // IMU filter Update run every 0.002 seconds
    //imuTimer.begin(updateIMU_filter, 2000);  // IMU filter Update run every 0.002 seconds
    
    
    // Setup voltage Reference pin as Output
    //pinMode(VoltRefPin, INPUT);

    // Setup LED pins as Outputs
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT); 
    pinMode(defaultLedPin, OUTPUT); 

    // Setup Left LoadCell pins as Outputs
    
    pinMode(FrcX1Pin, INPUT);
    pinMode(FrcY1Pin, INPUT);
    pinMode(FrcZ1Pin, INPUT);

    // Setup Right LoadCell pins as Outputs

    /*pinMode(FrcX2Pin, INPUT);
    pinMode(FrcY2Pin, INPUT);
    pinMode(FrcZ2Pin, INPUT);*/
    
    Serial.begin(115200);

    ////// ADC0 /////

    adc->adc0->setAveraging(16); // set number of averages
    adc->adc0->setResolution(16); // set bits of resolution
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
        
    /*adc->setAveraging(1); // set number of averages
    adc->setResolution(10); // set bits of resolution
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // ,  change the conversion speed
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed
    //adc->startContinuous(FrcX1Pin);
    //adc->startContinuous(FrcY1Pin);
    //adc->startContinuous(FrcZ1Pin);*/   

    
    /////////attachInterrupt(digitalPinToInterrupt(buttonPin), ButtonState, RISING);
    //MsTimer2::set(1, ResetFlashActivated);

    //FlexiTimer2::set(10, 1.0/1000, QSTMprint); // call every 10 1ms "ticks"

    if (!sd.begin(SD_CONFIG)) {      
      sd.initErrorHalt(&Serial);
      return;
    }
  
    //Print Card Info
  
    if (!sd.card()->readCID(&m_cid)) {
      error("CardReadInfo failed\n");
     // errorPrint();
      return;
    }
    
    while(!Serial){
      RGB_OFF();// Led off
      //RGB_Color(0,0,0); 
      delay(1000);
      RGB_WHITE();      
      delay(1000);
      continue; // wait for serial port to connect. Needed for native USB.  
    }


    Serial.println("Force-Motion DAQ");
  Serial.println("Palpation Device");
  while(inStr!=" "){
    if(Serial.available()){
      //inStr=Serial.readString();
      inStr=Serial.readStringUntil('\n');
      if(inStr.equalsIgnoreCase("DevType")){DeviceType(); yield();}
      if(inStr.equalsIgnoreCase("QSrNo")){ readSerialNumber(); yield();}
      if(inStr.equalsIgnoreCase("DevInfo")){readDeviceInfoFile(); yield();} 
      if(inStr.equalsIgnoreCase("HWInfo")){readHardwareInfoFile(); yield();} 
      if(inStr.equalsIgnoreCase("ReadRoot")){readRootDirectory(); yield();}      
      if(inStr.equalsIgnoreCase("ACK")){break;}           
    }
    else{
      Serial.println("NACK");
      idleModeBlink();
      //delay(1000);
      //break;
      continue;        
    }    
  }
  Serial.println();
  delay(1000);  
  Serial.println("Force-Motion DAQ Started");
  Serial.println();

  Wire.setClock(400000);
  Serial.println("Starting ICM");
  delay(10);
  i2c_scan();
  if (ICM_found)
  {
      icm20948.init(icmSettings);
  }
  
}



void loop() {
    
    elapsedMicros elapsMicro;
    currentMillis = millis();
    currentMicros = micros();
    // Read Actual Force Sensor Values
            
    //Calculate the force voltage Offsets
    //if(arrCnt>=sampleSize){arrCnt=0;}
    if (calibFlag == 1){// one time Calibration.
      Serial.println(F("Calibrating Load Cell..."));
      double *frcOffsetVolts;
      frcOffsetVolts = calculateVoltageOffsets();
      for(int b=0;b<3;b++){
        Voff[b]= *(frcOffsetVolts+b);      
      } 
      Serial.println (F("Offset Voltage"));     
      sprintf(QSTMString," %0.4f,%0.4f,%0.4f", *(frcOffsetVolts+0), *(frcOffsetVolts+1), *(frcOffsetVolts+2));
      //sprintf(QSTMString,"%0.4f,%0.4f,%0.4f, ,%0.4f,%0.4f,%0.4f", Vx1,Vy1,Vz1,Vx2,Vy2,Vz2);
      Serial.print (QSTMString);
      Serial.print (F(" micros "));
      Serial.println (elapsMicro);
      Serial.println();

      
      Serial.println(F("Load Cell Calibration-Done"));
      //RGB_OFF();// Led off

      Serial.println(F("Palpation Device is ready for Force-Motion sensing."));
      //DeviceStart();
      Serial.println(F("Start Device"));
      
      Serial.println();      
      calibFlag = 0; 

      startMillis = millis(); 
      //printTimer.begin(QSTMprint, 10000); // every 100ms
      //FlexiTimer2::set(10, 1.0/1000, QSTMprint); // call every 10 1ms "ticks"

      eventTimer.begin([] {MainCallBack();}, 2'000); // 2ms or 500Hz 
      //eventTimer.begin([] {MainCallBack();}, 5'000); // 5ms or 200Hz 
      imuTimer.begin([] {MotionCallBack();}, 5'000); // 5ms or 200Hz 
      //delay (100);
      printTimer.begin([] {QSTMprint();}, 10'000); // 10ms or 100Hz
       
      //FlexiTimer2::start();
      RGB_WHITE();// Led White 
      RGB_BLUE();               
    }
    //..............................Treatment Mode (Force Quantification & Tilt Sensing).................................................

    //RGB_OFF();// Led off

    //if ((unsigned long)(currentMicros - previousMicros) >= readForceInterval) {

      //...................Read ICM Sensor Readings (Accel-Ax,Ay,Az), Gyro(Gx, Gy, Gz), Magneto(Mx,My,Mz)................

      //............................Force Quantification Steps............................................
     //}
        
//CPU Restart
    if(!Serial){
      //FlexiTimer2::stop();
      //printTimer.end();
      eventTimer.stop();
      printTimer.stop();
      //imuTimer.stop();
      //noInterrupts();
      for(int ComEnd =0;ComEnd<=10;ComEnd++){
        //RGB_WHITE;
        RGB_OFF();
        RGB_RED();//Red
        delay(50);
        //RGB_WHITE();
        RGB_OFF();
        RGB_BLUE();//Blue         
        delay(50);             
      }
      CPU_RESTART
      delay(1000);
    }
    
}

void MotionCallBack(){

  //............................Read Accel, Gyro, Mag, AHRS values from ICM-20948............................................

      if (ICM_found){

         //if ((loopCnt >= 10)&&(loopCnt%5==0)) {
          
            icm20948.task();         
                       
            if (icm20948.accelDataIsReady()){icm20948.readAccelData(&Ax, &Ay, &Az);}
            if (icm20948.gyroDataIsReady()){icm20948.readGyroData(&Gx, &Gy, &Gz);}
            if (icm20948.magDataIsReady()){icm20948.readMagData(&Mx, &My, &Mz);}//QnW, QnX, QnY, QnZ
            //else if (icm20948.quat6DataIsReady()){icm20948.readQuat6Data(&quat_w, &quat_x, &quat_y, &quat_z);}
            //else if (icm20948.quat9DataIsReady()){icm20948.readQuat9Data(&quat_w, &quat_x, &quat_y, &quat_z);}
            if (icm20948.euler6DataIsReady()){icm20948.readEuler6Data(&icm_Roll, &icm_Pitch, &icm_Yaw);}
            //if (icm20948.euler9DataIsReady()){icm20948.readEuler9Data(&icm_Roll, &icm_Pitch, &icm_Yaw);}
            
            accRMS = getResultant(Ax,Ay,Az); 
            Axg = (double)Ax/accRMS;
            Ayg = (double)Ay/accRMS;
            Azg = (double)Az/accRMS;
            accRMS = getResultant(Axg,Ayg,Azg);
            gyroRMS = getResultant(Gx,Gy,Gz); 

            noInterrupts();

            imuParam[0]= Axg; imuParam[1]= Ayg; imuParam[2]= Azg; 
            imuParam[3]= Gx; imuParam[4]= Gy; imuParam[5]= Gz; 
            imuParam[6]= Mx; imuParam[7]= My; imuParam[8]= Mz;
            imuParam[9]= icm_Yaw; imuParam[10]= icm_Pitch; imuParam[11]= icm_Roll;

            interrupts();
           
          //}
      }
  
}

void MainCallBack(){

  //............................Read Force Values from the Load cells............................................


      //double *frcVolts;
      frcVolts = readSensorVoltage(FrcX1Pin, FrcY1Pin, FrcZ1Pin);
  
      for(int b=0;b<3;b++){
        FVolts[b]= *(frcVolts+b);      
      }

      arrCnt = loopCnt%bufferLength;  
      /*if((loopCnt%500==0)&&(resetFlag==1)){
        Serial.println(F("Int_Button, interrupt attached"));
        attachInterrupt(digitalPinToInterrupt(buttonPin), ButtonState, HIGH);  
        //resetFlag=2;  
      }*/
  
      //double *Volts2Frc;
      //Volts2Frc = Voltage2ForceTransform (FVolts, Voff, arrCnt);

      //...................................... Voltage rise in Left load cell .........................

      vXL = FVolts[0]-Voff[0];
      vYL = FVolts[1]-Voff[1];
      vZL = FVolts[2]-Voff[2]; 

      Vx1Val[arrCnt]= vXL;
      Vy1Val[arrCnt]= vYL;
      Vz1Val[arrCnt]= vZL;

      vXLavg=getAverage(Vx1Val, sampleSize);
      vYLavg=getAverage(Vy1Val, sampleSize);
      vZLavg=getAverage(Vz1Val, sampleSize);

      // Force transformed in Left Load cell 
      
      FHLx = (FCL11*vXLavg)+(FCL12*vYLavg)+(FCL13*vZLavg);// Calibration of the voltage components to the corresponding raw values
      FHLy = (FCL21*vXLavg)+(FCL22*(vYLavg+wt_volt_err))+(FCL23*vZLavg);// using the Calibration matrix.
      FHLz = (FCL31*vXLavg)+(FCL32*vYLavg)+(FCL33*vZLavg);
      /*FHLx = (FCL11*vXLavg)+(FCL12*vYLavg)+(FCL13*vZLavg);// Calibration of the voltage components to the corresponding raw values
      FHLy = (FCL21*vXLavg)+(FCL22*vYLavg)+(FCL23*vZLavg);// using the Calibration matrix.
      FHLz = (FCL31*vXLavg)+(FCL32*vYLavg)+(FCL33*vZLavg);*/

      /*//Force Scaling to Exact Newtons for left Load Cell.
      if(FHLx>0.0){FHLx=1.0*FHLx;}// Fx Positive
      if(FHLx<0.0){FHLx=1.0*FHLx;}// Fx Negative  
      if(FHLy>0.0){FHLx=1.0*FHLx;}// Fy Positive
      if(FHLy>0.0){FHLx=1.0*FHLx;}// Fy Negative
      if(FHLz>0.0){FHLx=1.0*FHLx;}// Fz Positive*/

      FHLrms = getResultant(FHLx, FHLy, FHLz);

      //MotionCallBack();     

      //KinematicWeightTransform(imuParam[9], imuParam[10], imuParam[11], &FHLx,&FHLy,&FHLz,&FHLrms);

      noInterrupts();

      //imuParam[0]= Ax; imuParam[1]= Ay; imuParam[2]= Az; 
      //imuParam[3]= Gx; imuParam[4]= Gy;imuParam[5]= Gz; 
      //imuParam[6]= Mx; imuParam[7]= My;imuParam[8]= Mz;
      
      printString[0] = FHLx; printString[1] = FHLy; printString[2] = FHLz; printString[3] = FHLrms;
      //printString[4] = Yaw; printString[5] = Pitch; printString[6] = Roll; 
      //printString[4] = icm_Yaw; printString[5] = icm_Pitch; printString[6] = icm_Roll; 
      //printString[7] = Axg; printString[8] = Ayg; printString[9] = Azg; printString[10] = accRMS;  
      //printString[11] = gyroRMS; printString[12] = Gx; printString[13] = Gy; printString[14] = Gz;
      //printString[15] = Mx; printString[16] = My; printString[17] = Mz;

      printString[4] = imuParam[9]; printString[5] = imuParam[10]; printString[6] = imuParam[11];
      printString[7] = imuParam[0]; printString[8] = imuParam[1]; printString[9] = imuParam[2]; printString[10] = getResultant(imuParam[0],imuParam[1],imuParam[2]);  
      printString[11] = getResultant(imuParam[3],imuParam[4],imuParam[5]); printString[12] = imuParam[3]; printString[13] = imuParam[4]; printString[14] = imuParam[5];
      printString[15] = imuParam[6]; printString[16] = imuParam[7]; printString[17] = imuParam[8];
      
      interrupts();
        
                  
      //previousMicros = micros();

      loopCnt++;  
  
}



void DeviceStart(){
  Serial.println(F("Start Device"));
  resetFlag=-1;
  while(buttonCount<0){
    RGB_GREEN();
    QSTMprint(); 
    if(!Serial){
      //FlexiTimer2::stop();
      //printTimer.end();
      eventTimer.stop();
      printTimer.stop();
      //imuTimer.stop();
      //noInterrupts();
      for(int ComEnd =0;ComEnd<=10;ComEnd++){
        //RGB_WHITE;
        RGB_OFF();
        RGB_RED();//Red
        delay(50);
        //RGB_WHITE();
        RGB_OFF();
        RGB_BLUE();//Blue         
        delay(50);              
      }
      CPU_RESTART
      delay(1000);
    }    
  }
  
}

/*void updateIMU_filter(){
  if (resetFlag == 0){
    //....................................GeoAngles estimation Code using Madgwick AHRS Filter .................................................
    //madgwickAHRS.updateIMU(myIMU.gx, myIMU.gy, myIMU.gz, Ax, Ay, Az);
    madgwickAHRS.updateIMU(imuParam[3], imuParam[4], imuParam[5], imuParam[0], imuParam[1], imuParam[2]);
    //madgwickAHRS.update(imuParam[3], imuParam[4], imuParam[5], imuParam[0], imuParam[1], imuParam[2],imuParam[6], imuParam[7], imuParam[8]);
    QnW = madgwickAHRS.getQnW(); QnX = madgwickAHRS.getQnX(); QnY = madgwickAHRS.getQnY(); QnZ = madgwickAHRS.getQnZ();
    Yaw = madgwickAHRS.getYawRadians();
    Pitch = madgwickAHRS.getPitchRadians();
    Roll = madgwickAHRS.getRollRadians();
    //Yaw = madgwickAHRS.getYaw();
    //Pitch = madgwickAHRS.getPitch();
    //Roll = madgwickAHRS.getRoll();


    //....................................GeoAngles estimation Code using Mahony AHRS Filter .................

    //mahonyAHRS.updateIMU(myIMU.gx, myIMU.gy, myIMU.gz, Ax, Ay, Az);
    //mahonyAHRS.updateIMU(imuParam[3], imuParam[4], imuParam[5], imuParam[0], imuParam[1], imuParam[2]);
    //mahonyAHRS.update(imuParam[3], imuParam[4], imuParam[5], imuParam[0], imuParam[1], imuParam[2],imuParam[6], imuParam[7], imuParam[8]);
    //QnW = mahonyAHRS.getQnW(); QnX = mahonyAHRS.getQnX(); QnY = mahonyAHRS.getQnY(); QnZ = mahonyAHRS.getQnZ();
    //Yaw = mahonyAHRS.getYawRadians();
    //Pitch = mahonyAHRS.getPitchRadians();
    //Roll = mahonyAHRS.getRollRadians();

     //compute euler angles from |quaternionFilters.h|...................................

    /*eulerAng = computeEulerAngles(QnW,QnX,QnY,QnZ);

    //Yaw = *(eulerAng+0);
    //Pitch = *(eulerAng+1);
    //Roll = *(eulerAng+2);
        
  }
  else{
    return;
  }
}*/

/*double *computeEulerAngles(double q0,double q1,double q2, double q3)
{
  double imuYaw,imuPitch,imuRoll;
  static double Angles[3];
  imuYaw = atan2(2.0f*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.29578f+170.0f;
  imuPitch = asin(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
  imuRoll = atan2(2.0f*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2+ q3*q3) * 57.29578f; 

  //yaw = atan2(2.0f*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
  //pitch = asin(-2.0f * (q1*q3 - q0*q2));
  //roll = atan2(2.0f*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2+ q3*q3); 

  ////pitch *= RAD_TO_DEG;
  //yaw   *= RAD_TO_DEG;
  // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
  //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  //yaw   -= 8.5;
  //roll  *= RAD_TO_DEG; 
  Angles[0] = imuYaw;
  Angles[1] = imuPitch;
  Angles[2] = imuRoll;

  return Angles;
  
}*/
/*
 //Rotation Matrix and Kinematic Transformation for Localized Device
   
   float *getRotationMatrix (float Psi,float Theta, float Phi){

  static float rotMat[9];

  //Convert Angles in degrees to radians
  Psi= Psi*0.0174533;
  Theta = Theta*0.0174533;
  Phi=Phi*0.0174533;
  
  BLA::Matrix<3, 3> RotX = {1, 0, 0,  0, cos(Phi), -sin(Phi), 0, sin(Phi), cos(Phi)};
  BLA::Matrix<3, 3> RotY = {cos(Theta), 0, sin(Theta),  0, 1, 0, -sin(Theta), 0, cos(Theta)};
  BLA::Matrix<3, 3> RotZ = {cos(Psi), -sin(Psi), 0,  sin(Psi), cos(Psi), 0,  0, 0, 1};

  BLA::Matrix<3, 3> RotYXZ = RotY*(~RotX)*(~RotZ);
  //rotMat = {RotYXZ(0,0), RotYXZ(0,1), RotYXZ(0,2),RotYXZ(1,0),RotYXZ(1,1),RotYXZ(1,2),RotYXZ(2,0),RotYXZ(2,1),RotYXZ(2,2)};
  rotMat[0] = RotYXZ(0,0); rotMat[1] = RotYXZ(0,1); rotMat[2] = RotYXZ(0,2);
  rotMat[3] = RotYXZ(1,0); rotMat[4] = RotYXZ(1,1); rotMat[5] = RotYXZ(1,2);
  rotMat[6] = RotYXZ(2,0); rotMat[7] = RotYXZ(2,1); rotMat[8] = RotYXZ(2,2);
  return rotMat;
}

void KinematicWeightTransform(float yawZ, float pitchY, float rollX, double *F3D_x,double *F3D_y, double *F3D_z, double *F3D_rms){

  //Based on orientation of Q1 the transformation axis are different
  //Hence the orientations are :  
  // or matrix multiplication. 
  //  [0  0 1    [Y    [R
  //   -1 0 0  *  P  =  -Y
  //   0 -1 0]    R]    P]
  // therefore angX = pitch; angY = -yaw; angZ = roll;

  //Rotation Matrix is YX`Z`. 
  
  arrMat = getRotationMatrix(rollX, -yawZ, pitchY);
  

  BLA::Matrix<3, 1> F_vec = {*F3D_x,  *F3D_y,  *F3D_z};
  
  BLA::Matrix<3, 1> F_wt = {0,  0.5,  0};

  BLA::Matrix<3, 3>rotMatrix = {*(arrMat+0),*(arrMat+1),*(arrMat+2),*(arrMat+3),*(arrMat+4),*(arrMat+5),*(arrMat+6),*(arrMat+7),*(arrMat+8)};
  
  BLA::Matrix<3, 1>F_Intertial = rotMatrix * F_vec;
  F_Intertial = F_Intertial - F_wt;// Subtract the weight (0.5N)in Y positive direction.
  F_vec = (~rotMatrix) * F_Intertial;
  *F3D_x = F_vec(0);
  *F3D_y = F_vec(1); 
  *F3D_z = F_vec(2);
  //F3Drms = getResultant(F3Dx, F3Dy, F3Dz);
  *F3D_rms = getResultant(*F3D_x, *F3D_y, *F3D_z);  
}
*/

/*
 * //Rotation Matrix and Kinematic Transformation for Dispersive Device
 * 
 * float *getRotationMatrix (float Psi,float Theta, float Phi){

  static float rotMat[9];

  //Convert Angles in degrees to radians
  Psi= Psi*0.0174533;
  Theta = Theta*0.0174533;
  Phi=Phi*0.0174533;
  
  BLA::Matrix<3, 3> RotX = {1, 0, 0,  0, cos(Phi), -sin(Phi), 0, sin(Phi), cos(Phi)};
  BLA::Matrix<3, 3> RotY = {cos(Theta), 0, sin(Theta),  0, 1, 0, -sin(Theta), 0, cos(Theta)};
  BLA::Matrix<3, 3> RotZ = {cos(Psi), -sin(Psi), 0,  sin(Psi), cos(Psi), 0,  0, 0, 1};

  BLA::Matrix<3, 3> RotZYX = RotZ*(~RotY)*RotX;
  //rotMat = {RotZYX(0,0), RotZYX(0,1), RotZYX(0,2),RotZYX(1,0),RotZYX(1,1),RotZYX(1,2),RotZYX(2,0),RotZYX(2,1),RotZYX(2,2)};
  rotMat[0] = RotZYX(0,0); rotMat[1] = RotZYX(0,1); rotMat[2] = RotZYX(0,2);
  rotMat[3] = RotZYX(1,0); rotMat[4] = RotZYX(1,1); rotMat[5] = RotZYX(1,2);
  rotMat[6] = RotZYX(2,0); rotMat[7] = RotZYX(2,1); rotMat[8] = RotZYX(2,2);
  return rotMat;
}

void KinematicWeightTransform(float yawZ, float pitchY, float rollX, double *F3D_x,double *F3D_y, double *F3D_z, double *F3D_rms){

  arrMat = getRotationMatrix(yawZ,pitchY,rollX);

  BLA::Matrix<3, 1> F_vec = {*F3D_x,  *F3D_y,  *F3D_z};
  
  BLA::Matrix<3, 1> F_wt = {0,  0,  blade_wt};

  BLA::Matrix<3, 3>rotMatrix = {*(arrMat+0),*(arrMat+1),*(arrMat+2),*(arrMat+3),*(arrMat+4),*(arrMat+5),*(arrMat+6),*(arrMat+7),*(arrMat+8)};
  
  BLA::Matrix<3, 1>F_Intertial = rotMatrix * F_vec;
  F_Intertial = F_Intertial + F_wt;
  F_vec = (~rotMatrix) * F_Intertial;
  *F3D_x = F_vec(0);
  *F3D_y = F_vec(1); 
  *F3D_z = F_vec(2);
  //F3Drms = getResultant(F3Dx, F3Dy, F3Dz);
  *F3D_rms = getResultant(*F3D_x, *F3D_y, *F3D_z);  
}*/


/*void ButtonState(){
  currentMillis=millis();
  buttonVal = digitalRead(buttonPin);
  elapsTime = (double)(currentMillis - startMillis)/1000.0;
  if (buttonVal>lastButtonVal){buttonCount++;}
  //Serial.print(F("ButtonVal : "));
  //Serial.println(buttonVal);
  //sprintf(QSTMString,"R:Q1_%0.3f, Button Val %d, Last Button Val %d, Button Count %d",elapsTime, buttonVal,lastButtonVal,buttonCount);
  //Serial.println(QSTMString);
  Serial.print(F("R:Q1_"));
  Serial.print(elapsTime);
  Serial.print(", Button Val ");
  Serial.print(buttonVal);
  Serial.print(" Last Button Val ");
  Serial.print(lastButtonVal);
  //Serial.print(" ");
  Serial.print(" Button Count ");
  Serial.println(buttonCount);
  delay (50);// 50 milliseconds;
  lastButtonVal=buttonVal; 
}*/

void QSTMprint(){  
      buttonVal = digitalRead(buttonPin);
      
      nowMillis = millis();
      elapsTime = (double)(nowMillis - startMillis)/1000.0;
      if (buttonVal>lastButtonVal){
        buttonCount++;
        sprintf(QSTMString,"R:Q1_%0.3f, Button Val %d, Last Button Val %d, Button Count %d",elapsTime, buttonVal,lastButtonVal,buttonCount);
        Serial.println(QSTMString);
        lastButtonVal=buttonVal;
        return;
      } 
      if((buttonCount%2==1)&&(buttonCount>0)){
        resetFlag=1;
      }
      else if (buttonCount%2==0){
        resetFlag=0;
        //RGB_WHITE();
        RGB_OFF();
        RGB_BLUE();
      }
      if(resetFlag<0){
        return;
      }      
      else if(resetFlag==0){

        /*printString[0] = FHLx; printString[1] = FHLy; printString[2] = FHLz; printString[3] = FHLrms;
        //printString[4] = Yaw; printString[5] = Pitch; printString[6] = Roll; 
        printString[4] = icm_Yaw; printString[5] = icm_Pitch; printString[6] = icm_Roll; 
        printString[7] = Axg; printString[8] = Ayg; printString[9] = Azg; printString[10] = accRMS;  
        printString[11] = gyroRMS; printString[12] = Gx; printString[13] = Gy; printString[14] = Gz;
        printString[15] = Mx; printString[16] = My; printString[17] = Mz;*/

         //-----------------------Print statement for Q-ware (Communication Protocol)--------------------------------------

        sprintf(QSTMString,"T:Q1_%0.3f,%0.2f,%0.2f,%0.2f,%0.2f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.2f,%d,%0.2f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f",elapsTime,printString[0],printString[1],printString[2],printString[3],printString[4],printString[5],printString[6],printString[7],printString[8],printString[9],printString[10],buttonCount,printString[11],printString[12],printString[13],printString[14],printString[15],printString[16],printString[17]);

        //sprintf(QSTMString,"T:Q1_%0.3f,Fx %0.2f,Fy %0.2f,Fz %0.2f,Frms %0.2f,Yz %0.2f,Py %0.2f,Rx %0.2f,Ax %0.2f,Ay %0.2f,Az %0.2f,Arms %0.2f,rst %d,Grms %0.2f,Gx %0.2f,Gy %0.2f,Gz %0.2f,Mx %0.2f,My %0.2f,Mz %0.2f",elapsTime,printString[0],printString[1],printString[2],printString[3],printString[4],printString[5],printString[6],printString[7],printString[8],printString[9],printString[10],buttonCount,printString[11],printString[12],printString[13],printString[14],printString[15],printString[16],printString[17]);

        //-----------------------------------------END of (Communication Protocol)--------------------------------------


        //sprintf(QSTMString,"T:Q1_%0.3f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%d,%0.2f",elapsTime,printString[0],printString[1],printString[2],printString[3],printString[4],printString[5],printString[6],printString[7],printString[8],printString[9],printString[10],buttonCount,printString[11]);

        //sprintf(QSTMString,"T:Q1_%0.3f,Fx %0.2f,Fy %0.2f,Fz %0.2f,Frms %0.2f,Yz %0.2f,Px %0.2f,Ry %0.2f,Ax %0.2f,Ay %0.2f,Az %0.2f,Arms %0.2f,rst %d,Grms %0.2f",elapsTime,printString[0],printString[1],printString[2],printString[3],printString[4],printString[5],printString[6],printString[7],printString[8],printString[9],printString[10],buttonCount,printString[11]);
        //sprintf(QSTMString,"T:Q1_%0.3f buttoncnt %d", elapsTime,buttonCount);
        //sprintf(QSTMString,"%0.4f,%0.4f,%0.4f, ,%0.4f,%0.4f,%0.4f", FVolts[0],FVolts[1],FVolts[2],FVolts[3],FVolts[4],FVolts[5]);
        Serial.println (QSTMString);
        //Serial.print (F(" micros "));
        //Serial.print (elapsMicro);
        //Serial.print (F(" buttonCnt "));
        //Serial.println ();       
      }
      else if(resetFlag==1){
        sprintf(QSTMString,"I:Q1_%0.2f, Reset Interval micros %d buttonCnt %d", elapsTime, elapsMicro, buttonCount);
        Serial.println (QSTMString);
        //Serial.print (F(" micros "));
        //Serial.print (elapsMicro);
        //Serial.print (F(" buttonCnt "));
        //Serial.println (buttonCount); 

        if ((int)(elapsTime)%2 == 0){RGB_WHITE();}
        else if ((int)(elapsTime)%2 == 1){RGB_MAGENTA();}
        //if ((previousMillis - startMillis)%1000 == 200){RGB_Color(255,255,255);}
         
      }     
      lastButtonVal=buttonVal;
        
}

double *calculateVoltageOffsets(){
  double *frcVolts,VoltRef;
  unsigned long cMicros=0, pMicros=0;
  //frcVolts = readForceVoltage(FrcX1Pin, FrcY1Pin, FrcZ1Pin, FrcX2Pin, FrcY2Pin, FrcZ2Pin);
  static double frcVoltOffset[3] = {0.0,0.0,0.0};
  //double RzOffset;
  uint16_t n=0, cnt = 2000;

  //RGB_Color(240,40,0);// Orange color
  RGB_RED();// Red color
  

  //VoltRef = readInputVoltRef(VoltRefPin);
  //Serial.print(F("Voltage Reference to Load Cells: "));
  //Serial.println(VoltRef);
  while(n < cnt){
    cMicros = micros();
    if ((unsigned long)(cMicros - pMicros) >= readForceInterval) {

      frcVolts = readSensorVoltage(FrcX1Pin, FrcY1Pin, FrcZ1Pin);//Reading Data from the LoadCells 
      avgVx1 += *(frcVolts+0); avgVy1 += *(frcVolts+1); avgVz1 += *(frcVolts+2);
      //avgVx2 += *(frcVolts+3); avgVy2 += *(frcVolts+4); avgVz2 += *(frcVolts+5);
      
      //LEFT Load Cell voltage NOISE Detection 
      if (*(frcVolts+0) > MaxVLx) { MaxVLx = *(frcVolts+0);}    
      if (*(frcVolts+0) < MinVLx) { MinVLx = *(frcVolts+0);}
      //Max and min Vy  
      if (*(frcVolts+1) > MaxVLy) {MaxVLy = *(frcVolts+1);}    
      if (*(frcVolts+1) < MinVLy) {MinVLy = *(frcVolts+1);}
      //Max and min Vz
      if (*(frcVolts+2) > MaxVLz) {MaxVLz = *(frcVolts+2);}    
      if (*(frcVolts+2) < MinVLz) {MinVLz = *(frcVolts+2);}

      /*//RIGHT Load Cell voltage NOISE Detection
      if (*(frcVolts+3) > MaxVRx) { MaxVRx = *(frcVolts+3);}    
      if (*(frcVolts+3) < MinVRx) { MinVRx = *(frcVolts+3);}
      //Max and min Vy  
      if (*(frcVolts+4) > MaxVRy) {MaxVRy = *(frcVolts+4);}    
      if (*(frcVolts+4) < MinVRy) {MinVRy = *(frcVolts+4);}
      //Max and min Vz
      if (*(frcVolts+5) > MaxVRz) {MaxVRz = *(frcVolts+5);}    
      if (*(frcVolts+5) < MinVRz) {MinVRz = *(frcVolts+5);}*/
          
      pMicros = micros();
      n++;
    }
      
  }
  Serial.print("MaxVLy ");  Serial.println(MaxVLy);
  Serial.print("MinVLy ");  Serial.println(MinVLy);
   
  avgVx1= avgVx1/cnt; avgVy1=avgVy1/cnt; avgVz1=avgVz1/cnt; 
  //avgVx2= avgVx2/cnt; avgVy2=avgVy2/cnt; avgVz2=avgVz2/cnt;
  Serial.println (F("Average Offset Voltage (V)"));
  //sprintf(CalibString,"%0.4f,%0.4f,%0.4f, ,%0.4f,%0.4f,%0.4f", frcVoltOffset[0],frcVoltOffset[1],frcVoltOffset[2],frcVoltOffset[3],frcVoltOffset[4],frcVoltOffset[5]);
  //Serial.println (CalibString);
  sprintf(CalibString,"%0.4f,%0.4f,%0.4f",avgVx1,avgVy1,avgVz1);
  Serial.println (CalibString);
  
  
  // Median Offset voltage Values
  MedianVLx = MinVLx + (MaxVLx-MinVLx)/2;    MedianVLy = MinVLy + (MaxVLy-MinVLy)/2;    MedianVLz = MinVLz + (MaxVLz-MinVLz)/2;
  //MedianVRx = MinVRx + (MaxVRx-MinVRx)/2;    MedianVRy = MinVRy + (MaxVRy-MinVRy)/2;    MedianVRz = MinVRz + (MaxVRz-MinVRz)/2;
  Serial.println (F("Median Offset Voltage (V)"));
  sprintf(CalibString,"%0.4f,%0.4f,%0.4f", MedianVLx,MedianVLy,MedianVLz);
  Serial.println (CalibString);
  
  // Range of Offset voltage Values in Millivolts
  RangeVLx = (MaxVLx-MinVLx);    RangeVLy = (MaxVLy-MinVLy);   RangeVLz = (MaxVLz-MinVLz);
  //RangeVRx = (MaxVRx-MinVRx);    RangeVRy = (MaxVRy-MinVRy);   RangeVRz = (MaxVRz-MinVRz);
  Serial.println (F("Voltage Noise Range (mV)"));
  sprintf(CalibString,"%0.4f,%0.4f,%0.4f", RangeVLx,RangeVLy,RangeVLz);
  Serial.println (CalibString);
  
  //Force Transform Range of Offset voltage Values
  offsetFLx = (FCL11*RangeVLx)+(FCL12*RangeVLy)+(FCL13*RangeVLz);// Calibration of the voltage components to the corresponding raw values of
  offsetFLy = (FCL21*RangeVLx)+(FCL22*RangeVLy)+(FCL23*RangeVLz);// Left Load Cell using the Calibration matrix.
  offsetFLz = (FCL31*RangeVLx)+(FCL32*RangeVLy)+(FCL33*RangeVLz);

  /*offsetFRx = (FCR11*RangeVRx)+(FCR12*RangeVRy)+(FCR13*RangeVRz);// Calibration of the voltage components to the corresponding raw values
  offsetFRy = (FCR21*RangeVRx)+(FCR22*RangeVRy)+(FCR23*RangeVRz);// Right Load Cell using the Calibration matrix.
  offsetFRz = (FCR31*RangeVRx)+(FCR32*RangeVRy)+(FCR33*RangeVRz);*/

  Serial.println (F("Voltage-Force transformed Noise Range (Newtons)"));
  sprintf(CalibString,"%0.4f,%0.4f,%0.4f", offsetFLx,offsetFLy,offsetFLz);
  Serial.println (CalibString);
  
  Serial.println(F("Voltage Offsets Calculated"));
  Serial.println();
  //frcVoltOffset = {avgVx1,avgVy1,avgVz1,avgVx2,avgVy2,avgVz2};
  //if (MedianVRz>avgVz2){RzOffset=MedianVRz;}else{RzOffset=avgVz2;}

  frcVoltOffset[0]=avgVx1; frcVoltOffset[1]=avgVy1, frcVoltOffset[2]=avgVz1;
  //if (MedianVLz>avgVz1){frcVoltOffset[2]=MedianVLz;}else{frcVoltOffset[2]=avgVz1;}
  //frcVoltOffset[3]=avgVx2; frcVoltOffset[4]=avgVy2; frcVoltOffset[5]=avgVz2; 
  //if (frcVoltOffset[5]>avgVz2){frcVoltOffset[5]=MedianVRz;}else{frcVoltOffset[5]=avgVz2;}
  //frcVoltOffset[5]=RzOffset; 

  //RGB_WHITE();//White
  RGB_OFF();// Led off
  //RGB_Color(255,255,255);//White

  /*frcVoltOffset[0]=MedianVLx; frcVoltOffset[1]=MedianVLy, frcVoltOffset[2]=max(MedianVLz,avgVz1);
  //if (MedianVLz>avgVz1){frcVoltOffset[2]=MedianVLz;}else{frcVoltOffset[2]=avgVz1;}
  frcVoltOffset[3]=MedianVRx; frcVoltOffset[4]=MedianVRy; frcVoltOffset[5]=max(MedianVRz,avgVz2); 
  //if (frcVoltOffset[5]>avgVz2){frcVoltOffset[5]=MedianVRz;}else{frcVoltOffset[5]=avgVz2;}
  //frcVoltOffset[5]=RzOffset;*/ 
  return frcVoltOffset;
}

double *readSensorVoltage(int FrcX1pin, int FrcY1pin, int FrcZ1pin){
  //double forceVoltageVal[6];
  static double forceVoltageVal[3];
  //uint16_t XLval, YLval,Zlval, XRval, YRval, ZRval;
  /*analogReadResolution(10);

  forceVoltageVal[0] = (double)((uint16_t)analogRead(FrcX1pin)*5.0)/1023.0;
  forceVoltageVal[1] = (double)((uint16_t)analogRead(FrcY1pin)*5.0)/1023.0;
  forceVoltageVal[2] = (double)((uint16_t)analogRead(FrcZ1pin)*5.0)/1023.0;

  forceVoltageVal[3] = (double)((uint16_t)analogRead(FrcX2pin)*5.0)/1023.0;
  forceVoltageVal[4] = (double)((uint16_t)analogRead(FrcY2pin)*5.0)/1023.0;
  forceVoltageVal[5] = (double)((uint16_t)analogRead(FrcZ2pin)*5.0)/1023.0;*/
 
  //Serial.println(forceVoltageVal[2]);

  forceVoltageVal[0] = (((double)((uint16_t)adc->adc0->analogRead(FrcX1pin))*3.3)/adc->adc0->getMaxValue());
  forceVoltageVal[1] = (((double)((uint16_t)adc->adc0->analogRead(FrcY1pin))*3.3)/adc->adc0->getMaxValue());
  forceVoltageVal[2] = (((double)((uint16_t)adc->adc0->analogRead(FrcZ1pin))*3.3)/adc->adc0->getMaxValue());

  /*forceVoltageVal[3] = (((double)((uint16_t)adc->analogRead(FrcX2pin))*3.3)/adc->getMaxValue(ADC_1));
  forceVoltageVal[4] = (((double)((uint16_t)adc->analogRead(FrcY2pin))*3.3)/adc->getMaxValue(ADC_1));
  forceVoltageVal[5] = (((double)((uint16_t)adc->analogRead(FrcZ2pin))*3.3)/adc->getMaxValue(ADC_1));
  //Serial.println(forceVoltageVal[2]);
  //forceVoltageVal = {Vx1,Vy1,Vz1,Vx2,Vy2,Vz2};*/
  return forceVoltageVal;  
}

/*double readInputVoltRef(int VoltPin){

  int Cnt=20000;
  int VoltVal;
  double VoltageVal=0.0, VoltSum, VoltRefAvg;

  for (int i=0;i<Cnt;i++){
    VoltVal = analogRead(VoltPin);
    //VoltageVal = (((double)((uint16_t)adc->analogRead(VoltPin))*5.0)/adc->getMaxValue(ADC_1));
    Serial.println (VoltVal); 
    VoltSum += VoltageVal;
    delay(10);   
  }
  VoltRefAvg = VoltSum/Cnt; 

  return VoltRefAvg;  
}*/

// Function to calculate the average values of Force components
double getAverage(double arr[], int size) {

   int i;
   double avg;
   double sum = 0;

   for (i = 0; i < size; ++i) {
      sum += arr[i];
   }

   avg = sum / size;

   return avg;
}

double getResultant(double Xval, double Yval, double Zval){
  double XvalSq,YvalSq,ZvalSq,SumXYZ,RezXYZ;
  XvalSq = Xval*Xval;
  YvalSq = Yval*Yval;
  ZvalSq = Zval*Zval;
  SumXYZ = XvalSq+YvalSq+ZvalSq;
  RezXYZ = sqrt(SumXYZ);
  return RezXYZ;
  
}

void RGB_Color(int redVal, int greenVal, int blueVal){
  //Set Colors for Common anode Display
  RGB_OFF();
  analogWrite(redPin,(255 - redVal));
  analogWrite(greenPin,(255 - greenVal));
  analogWrite(bluePin,(255 - blueVal));
}

void RGB_WHITE(){RGB_Color(169,169,169);}
void RGB_RED(){RGB_Color(169,0,0);}
void RGB_GREEN(){RGB_Color(0,169,0);}
void RGB_BLUE(){RGB_Color(0,0,169);}
void RGB_MAGENTA(){RGB_Color(169,0,169);}

void RGB_OFF(){
  //Set Colors for Common anode Display
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT); 
  digitalWriteFast(redPin,HIGH);
  digitalWriteFast(greenPin,HIGH);
  digitalWriteFast(bluePin,HIGH);
}

void idleModeBlink(){
  RGB_OFF();// Led off
  delay(1000);
  RGB_WHITE();      
  delay(1000);  
}
void readRootDirectory(){
  
  int rootFileCount = 0;
  if (!root.open("/")) {
    error("open root");
  }
  while (file.openNext(&root, O_RDONLY)) {
    if (!file.isHidden()) {
      rootFileCount++;
    }
    file.close();
    if (rootFileCount > 10) {
      error("Too many files in root. Please use an empty SD.");
    }
  }
  Serial.print("\nRoot file count: ");
  Serial.println(rootFileCount);
  
  if (rootFileCount) {
    Serial.println(F("\nPlease use an empty SD for best results.\n\n"));
    delay(1000);
  }
   // Finally list the sd card files

  Serial.println(F("\nList of files on the SD.\n"));
  sd.ls("/", LS_R);
  
  
}

void readDeviceInfoFile(){

  if (!file.open("ProductInformation/DeviceInfo.txt", O_RDONLY)) {
    error("File read failed");
  }

//  if (!file.open("ProductInfo.txt", O_RDONLY)) {
//    error("File read failed");
//  }

  while (file.available()) {
    int n = file.fgets(line, sizeof(line));
    if (n <= 0) {
      error("fgets failed"); 
    }

    //Serial.println(line);
    
    if (line[n - 1] == '\n') {
      Serial.println(line);
    }
    else{continue;}     
  }
  file.close();
}

void readHardwareInfoFile(){

  if (!file.open("ProductInformation/HardwareInfo.txt", O_RDONLY)) {
    error("File read failed");
  }

//  if (!file.open("ProductInfo.txt", O_RDONLY)) {
//    error("File read failed");
//  }

  while (file.available()) {
    int n = file.fgets(line, sizeof(line));
    if (n <= 0) {
      error("fgets failed"); 
    }

    //Serial.println(line);
    
    if (line[n - 1] == '\n') {
      Serial.println(line);
    }
    else{continue;}     
  }
  file.close();
}

void readSerialNumber(){
  
  //Print Card Info

  if (!sd.card()->readCID(&m_cid)) {
    error("CardReadInfo failed\n");
   // errorPrint();
    return;
  } 
  //cidDmp();
  SrNo = m_cid.psn;
  //Serial.print(F("Serial Num (int): "));
  //Serial.println(SrNo,DEC);
  Serial.print(F("Serial Num (hex): "));
  Serial.println(SrNo,HEX);
}

void DeviceType(){
  Serial.print(F("Dev-Q1"));
  Serial.println();
  
}

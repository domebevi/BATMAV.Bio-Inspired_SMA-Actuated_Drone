#define TIMER_INTERRUPT_DEBUG 0
#define _TIMERINTERRUPT_LOGLEVEL_ 0

#include "ArduinoBLE.h"
#include "NRF52_MBED_TimerInterrupt.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include "Math.h"

#define pinRA D0
#define pinRB D1
#define pinLA D2
#define pinLB D3

//BLE CONNECTION VARIABLES
//------------------------------------------------------------------------------------------
BLEService mainService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
BLEByteCharacteristic cmdIDcharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8", BLERead | BLEWrite);
BLEByteCharacteristic cmdVALUEcharacteristic("6e1e74e1-435d-4c95-8dff-9e9651b77bfe", BLERead | BLEWrite);
BLEByteCharacteristic cmdMANUALcharacteristic("a16512fd-ce94-4245-8a6f-c97fa5c6fdeb", BLERead | BLEWrite);
BLEByteCharacteristic sensorPITCHcharacteristic("7a66f003-e2c3-4189-95b7-34608d313145", BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic sensorROLLcharacteristic("7f0c9e07-ef47-4590-b419-6df5fe9bf65a", BLERead | BLEWrite | BLENotify);

int cmdID = 0;
int cmdVALUE = 0;

//ON-BOARD SENSORS VARIABLES
//------------------------------------------------------------------------------------------
LSM6DS3 batIMU(I2C_MODE, 0x6A);
float elapsedTime, currentTime, previousTime;
float accX, accY, accZ, gyrX, gyrY, gyrZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float pitch, roll;
int loop_n = 0;
bool roll_control = false;
const int sensor_update_rate = 10;

void setMotionInterrupt(LSM6DS3 & imu) 
{
  imu.writeRegister(0x10, 0x60);
  imu.writeRegister(0x58, 0x90);
  imu.writeRegister(0x5C, 0x00);
  imu.writeRegister(0x5B, 0xD);
  imu.writeRegister(0x5E, 0x20);
}

//INPUT SIGNAL VARIABLES
//-------------------------------------------------------------------------------------------
bool flapping = false;
bool error_check = false;
float freq = 1;          

int R_A, R_B, L_A, L_B;
int amplitude_R_A = 100;  
int amplitude_R_B = 100;  
int amplitude_L_A = 100;  
int amplitude_L_B = 100;   

int dutyCycle_A = 20;    
int dutyCycle_B = 20; 

int turn_offset = 0; 

int manual_R, manual_L;
int manual_limit = 130;

//TIMERS
//-------------------------------------------------------------------------------------------
NRF52_MBED_Timer Timer_ON(NRF_TIMER_3);
NRF52_MBED_Timer Timer_OFF(NRF_TIMER_4);

volatile bool active_A = false;
volatile bool active_B = false;

unsigned long micros_A = round(1000L * 1000 * dutyCycle_A/ (100 * freq));
unsigned long micros_B = round(1000L * 1000 * dutyCycle_B/ (100 * freq));

void Timer_ONHandler(void)
{
  if(flapping)
  {
    active_A = !active_A;
    active_B = !active_A;
    Timer_OFF.setInterval(micros_A*active_A + micros_B*active_B, Timer_OFFHandler);
    
    analogWrite(pinRA, R_A*active_A);
    analogWrite(pinRB, R_B*active_B);
    analogWrite(pinLA, L_A*active_A);
    analogWrite(pinLB, L_B*active_B);
  }
  error_check = true;
}

void Timer_OFFHandler(void)
{
  if(flapping)
  {
    analogWrite(pinRA, 0);
    analogWrite(pinRB, 0);
    analogWrite(pinLA, 0);
    analogWrite(pinLB, 0);
  }
  error_check = false;
}

//SETUP
//-------------------------------------------------------------------------------------------
void setup() 
{
  pinMode(LEDB, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(pinRA, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinLA, OUTPUT);
  pinMode(pinLB, OUTPUT);

  digitalWrite(LEDG,LOW);
  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDR, HIGH);
  analogWrite(pinRA,0);
  analogWrite(pinRB,0);
  analogWrite(pinLA,0);
  analogWrite(pinLB,0);
  delay(200);

  BLE.begin();
  delay(200);
  
  mainService.addCharacteristic(cmdIDcharacteristic);
  mainService.addCharacteristic(cmdVALUEcharacteristic);
  mainService.addCharacteristic(cmdMANUALcharacteristic);
  mainService.addCharacteristic(sensorPITCHcharacteristic);
  mainService.addCharacteristic(sensorROLLcharacteristic);
  BLE.setLocalName("BATMAV");
  BLE.addService(mainService);
  BLE.setAdvertisedService(mainService);
  BLE.advertise();
  delay(200);

  batIMU.begin();
  delay(200);

  Timer_ON.setFrequency(freq*2, Timer_ONHandler);
  Timer_OFF.setInterval(micros_A, Timer_OFFHandler);
  Timer_ON.stopTimer();
  Timer_OFF.stopTimer();
  delay(200);

  R_A = amplitude_R_A;
  R_B = amplitude_R_B;
  L_A = amplitude_L_A;
  L_B = amplitude_L_B;

  digitalWrite(LEDG, HIGH);
}

//EXECUTE
//-------------------------------------------------------------------------------------------
void loop() 
{
  BLEDevice central = BLE.central();
  digitalWrite(LEDB,HIGH);
  digitalWrite(LEDR,LOW);

  if (central) 
  {
    digitalWrite(LEDB,LOW);
    digitalWrite(LEDR,HIGH);
    delay(1000);

    while (central.connected()) 
    {
      digitalWrite(LEDB,HIGH);
      digitalWrite(LEDR,HIGH);
      
      previousTime = currentTime;        
      currentTime = millis();            
      elapsedTime = (currentTime - previousTime) / 1000; 

      setMotionInterrupt(batIMU);
      accX = (batIMU.readRawAccelX() * 0.061);
      accY = (batIMU.readRawAccelY() * 0.061);
      accZ = (batIMU.readRawAccelZ() * 0.061);
      gyrX = batIMU.readRawGyroX() / 17.5;
      gyrY = batIMU.readRawGyroY() / 17.5;
      gyrZ = batIMU.readRawGyroZ() / 17.5;
  
      accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 /PI); 
      accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI);

      gyroAngleX = roll + gyrX * elapsedTime;
      gyroAngleY = pitch + gyrY * elapsedTime;

      roll = 0.9 * gyroAngleX + 0.1 * accAngleX;
      pitch = 0.9 * gyroAngleY + 0.1 * accAngleY;
  
      if(loop_n > sensor_update_rate)
      {
        sensorPITCHcharacteristic.writeValue(round(pitch+90));
        sensorROLLcharacteristic.writeValue(round(roll+90));
        loop_n = 0;
      }

      R_A = constrain(amplitude_R_A - turn_offset, 0, 255);
      R_B = constrain(amplitude_R_B - turn_offset, 0, 255);
      L_A = constrain(amplitude_L_A + turn_offset, 0, 255);
      L_B = constrain(amplitude_L_B + turn_offset, 0, 255);
      
      if (cmdIDcharacteristic.written()) 
        cmdID = cmdIDcharacteristic.value();

      if (cmdVALUEcharacteristic.written()) 
      {
        cmdVALUE = cmdVALUEcharacteristic.value();       
        
        switch (cmdID)
        {
          case 1:
            if(cmdVALUE == 0)
            {
              flapping = false;
              analogWrite(pinRA, 0);
              analogWrite(pinRB, 0);
              analogWrite(pinLA, 0);
              analogWrite(pinLB, 0);
            }
            else
            {
              flapping = true;
              freq = cmdVALUE;
              Timer_ON.setFrequency(2*freq, Timer_ONHandler);
              micros_A = round(1000L * 1000 * dutyCycle_A / (100 * freq));
              micros_B = round(1000L * 1000 * dutyCycle_B / (100 * freq));
            }
          break;

          case 2:
            dutyCycle_A = constrain(cmdVALUE, 1, 49);
            if(dutyCycle_A == 25)
              dutyCycle_A = 24;
            micros_A = round(1000 * 1000 * dutyCycle_A / (100 * freq));
            break;

          case 3:
            dutyCycle_B = constrain(cmdVALUE, 1, 49);
            if(dutyCycle_B == 25)
              dutyCycle_B = 24;
            micros_B = round(1000 * 1000 * dutyCycle_B / (100 * freq));
            break;

          case 4:
            amplitude_R_A = cmdVALUE;
          break;

          case 5:
            amplitude_R_B = cmdVALUE;
          break;

          case 6:
            amplitude_L_A = cmdVALUE;
          break;

          case 7:
            amplitude_L_B = cmdVALUE;
          break;

          case 10:
            if(cmdVALUE == 1)
              roll_control = true;
            else
              roll_control = false;
          break;
          
        }
      }
      
      if (sensorPITCHcharacteristic.written()) 
        gyroAngleY = 0;

      if (sensorROLLcharacteristic.written()) 
        gyroAngleX = 0;

      if (cmdMANUALcharacteristic.written()) 
      {
        turn_offset = round(2.5*(cmdMANUALcharacteristic.value() - 100)); 
        if(flapping == false)
        {
          manual_R = constrain(turn_offset, 0, manual_limit);
          manual_L = constrain(-turn_offset, 0, manual_limit);
          analogWrite(pinRA, manual_L);
          analogWrite(pinRB, manual_R);
          analogWrite(pinLA, manual_R);
          analogWrite(pinLB, manual_L); 
        }
      } 

      loop_n++;       
    }
    digitalWrite(LEDB,LOW);
    digitalWrite(LEDR,LOW);
    analogWrite(pinRA, 0);
    analogWrite(pinRB, 0);
    analogWrite(pinLA, 0);
    analogWrite(pinLB, 0);
    flapping = false;
    loop_n = 0;
  }
}


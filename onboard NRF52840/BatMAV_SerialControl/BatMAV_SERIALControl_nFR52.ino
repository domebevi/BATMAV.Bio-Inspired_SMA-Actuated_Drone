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

int cmdID = 0;
int cmdVALUE = 0;

//ON-BOARD SENSORS VARIABLES
//------------------------------------------------------------------------------------------
LSM6DS3 batIMU(I2C_MODE, 0x6A);
float elapsedTime, currentTime, previousTime;
float accX, accY, accZ, gyrX, gyrY, gyrZ;
float EaccX, EaccY, EaccZ, EgyrX, EgyrY, EgyrZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float pitch;
float roll;

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
float freq = 1;          

int amplitude_R_A = 50;  
int amplitude_R_B = 50;  
int amplitude_L_A = 50;  
int amplitude_L_B = 50;  

int turn_offset = 0;  

int dutyCycle_A = 20;    
int dutyCycle_B = 20;    

float manual_limit = 0.3;

int R_A, R_B, L_A, L_B;

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
  active_A = !active_A;
  active_B = !active_A;
  Timer_OFF.setInterval(micros_A*active_A + micros_B*active_B, Timer_OFFHandler);
  
  analogWrite(pinRA, R_A*active_A*flapping);
  analogWrite(pinRB, R_B*active_B*flapping);
  analogWrite(pinLA, L_A*active_A*flapping);
  analogWrite(pinLB, L_B*active_B*flapping);
}

void Timer_OFFHandler(void)
{
  analogWrite(pinRA, 0);
  analogWrite(pinRB, 0);
  analogWrite(pinLA, 0);
  analogWrite(pinLB, 0);
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

  Serial.begin(115200);
  delay(400);
  batIMU.begin();
  delay(400);

  R_A = amplitude_R_A;
  R_B = amplitude_R_B;
  L_A = amplitude_L_A;
  L_B = amplitude_L_B;

  Timer_ON.setFrequency(freq*2, Timer_ONHandler);
  Timer_OFF.setInterval(micros_A, Timer_OFFHandler);
  Timer_ON.stopTimer();
  Timer_OFF.stopTimer();
  delay(200);

  for (int i = 0; i < 100; i++)
  {
      EaccX += batIMU.readFloatAccelX();
      EaccY += batIMU.readFloatAccelY();
      EaccZ += batIMU.readFloatAccelZ();
      EgyrX += batIMU.readFloatGyroX();
      EgyrY += batIMU.readFloatGyroY();
      EgyrZ += batIMU.readFloatGyroZ();
  }

  EaccX = EaccX/100;
  EaccY = EaccY/100;
  EaccZ = EaccZ/100;
  EgyrX = EgyrX/100;
  EgyrY = EgyrY/100;
  EgyrZ = EgyrZ/100;
  
  digitalWrite(LEDG, HIGH);
}

//EXECUTE
//-------------------------------------------------------------------------------------------
void loop() 
{
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
  
  accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI); 
  accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI);

  gyroAngleX = roll + gyrX * elapsedTime;
  gyroAngleY = pitch + gyrY * elapsedTime;

  roll = 0.9 * gyroAngleX + 0.1 * accAngleX;
  pitch = 0.9 * gyroAngleY + 0.1 * accAngleY;
  
  Serial.println("roll: " + String(round(roll+90)) + " | pitch: " + String(round(pitch+90)));

  R_A = constrain(amplitude_R_A + turn_offset, 0, 255);
  R_B = constrain(amplitude_R_B + turn_offset, 0, 255);
  L_A = constrain(amplitude_L_A - turn_offset, 0, 255);
  L_B = constrain(amplitude_L_B - turn_offset, 0, 255);
  
  if (Serial.available() > 0) 
  {
    char command = Serial.read();
    int value = Serial.parseInt();

    switch (command)
    {         
      case 'F':
        if(value == 0)
          flapping = false;
        else
        {
          flapping = true;
          freq = value;
          Timer_ON.setFrequency(2*freq, Timer_ONHandler);
          micros_A = round(1000L * 1000 * dutyCycle_A / (100 * freq));
          micros_B = round(1000L * 1000 * dutyCycle_B / (100 * freq));
        }
        break;

      case 'X':
        dutyCycle_A = constrain(value, 1, 49);
        if(dutyCycle_A == 25)
          dutyCycle_A = 24;
        micros_A = round(1000L * 1000 * dutyCycle_A / (100 * freq));
        break;
    
      case 'Y':
        dutyCycle_B = constrain(value, 1, 49);
        if(dutyCycle_B == 25)
          dutyCycle_B = 24;
        micros_B = round(1000L * 1000 * dutyCycle_B / (100 * freq));
        break;

      case 'A':
        amplitude_R_A = value;
        break;

      case 'B':
        amplitude_R_B = value;
        break;

      case 'C':
        amplitude_L_A = value;
        break;

      case 'D':
        amplitude_L_B = value;
        break;

      case 'M':
        turn_offset = value;
        break;
    }
  }
}
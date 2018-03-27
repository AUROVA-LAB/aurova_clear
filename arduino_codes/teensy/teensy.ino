// Program for Teeensy 3.2
// Reading of two incremental quadrature encoders


#include <Wire.h> // I2C library
#include <elapsedMillis.h> // For scheduling tasks
#include <Snooze.h>

// Encoders signals
#define PIN_EN_A 5
#define PIN_EN_B 6
#define PIN_HALL 7
#define PIN_EN2_B 8


//----ENCODER 1 STEERING----//
float vel1   = 0.0;

//Pulses
volatile int32_t pulses1 = 0;
volatile int32_t pulses_prev1 = 0;

//Times
uint32_t dt = 0;
float dt_f = 0;
uint32_t currTime = 0;
uint32_t prevTime1 = 0;

//Interrupts
void encoder_count1(void)
{
  if(digitalReadFast(PIN_EN_B) == HIGH)
    pulses1 -= 1;
  else
    pulses1 += 1;
}

void ComputeState1()
{
  currTime = micros();
  if(currTime > prevTime1)
    dt = (float)(currTime-prevTime1);
  else
    dt = (float)(4294967295-prevTime1);  
  
  dt_f = dt/1000000.0;

  vel1  = (pulses1-pulses_prev1) / dt_f;

  pulses_prev1 = pulses1;
  prevTime1 = currTime; 
}

//----ENCODER 2 SPEED----//
float vel2   = 0.0;
float acel2   = 0.0;
float jerk2  = 0.0;
float vel_prev2   = 0.0;
float acel_prev2   = 0.0;

//Pulses;
volatile int32_t pulses2 = 0;
volatile int32_t pulses_prev2 = 0;

//Times
uint32_t prevTime2 = 0;

//Interrupts
void encoder_count2(void) {pulses2 += 1;}

void ComputeState2()
{
  currTime = micros();
  if(currTime > prevTime2)
    dt = (float)(currTime-prevTime2);
  else
    dt = (float)(4294967295-prevTime2); 
  
  dt_f = dt/1000000.0;
  
  vel2  = (pulses2-pulses_prev2) / dt_f;
  acel2 = (vel2-vel_prev2) / dt_f;
  jerk2 = (acel2-acel_prev2) / dt_f;

  pulses_prev2 = pulses2;
  vel_prev2 = vel2;
  acel_prev2   = acel2;
  prevTime2 = currTime;
}


//-------I2C-----//
// I2C configuration
#define I2C_SLAVEADDR 0x33  // Direction for this slave
#define I2C_SDA 18          // I2C SDA pin
#define I2C_SCL 19  // I2C SCL pin

int16_t I2C_sizeFloat = sizeof(float);
int16_t I2C_sizeInt = sizeof(int16_t);
byte I2C_sendBuffer[sizeof(float)];       // Sending buffer to store one or two long integers
byte I2C_receiveBuffer[1+sizeof(int16_t)];   // Reception buffer to store one byte for a command plus one long integer


void I2C_eventRequest(void) 
{
  float* pBuffer = (float*)I2C_sendBuffer;

  // Evaluates the command previously stored in the first byte of the reception buffer
  switch (I2C_receiveBuffer[0]) 
  {
    //Encoder 1 Steering
   case 0:
      ComputeState1();
      *pBuffer = (float)pulses_prev1;
      Wire.write(I2C_sendBuffer, I2C_sizeFloat);
      break;
  case 1:
      *pBuffer = vel1;
      Wire.write(I2C_sendBuffer, I2C_sizeFloat);
      break;
    
    //Encoder 2 Speed
  case 2:
      ComputeState2();
      *pBuffer = vel2;
      Wire.write(I2C_sendBuffer, I2C_sizeFloat);
      break;
  case 3:
      *pBuffer = acel2;
      Wire.write(I2C_sendBuffer, I2C_sizeFloat);
      break;
  case 4:
      *pBuffer = jerk2;
      Wire.write(I2C_sendBuffer, I2C_sizeFloat);
      break;
  }  
}

void I2C_eventReceive(int bytesReceived) 
{
  // Read the maximun expected number of bytes from the master
  int index=0;
  while ((index < bytesReceived) && (index < 1 + I2C_sizeInt))
    I2C_receiveBuffer[index++] = Wire.read();

  // Read and discard the possible remaining data
  while (index < bytesReceived)
    Wire.read();

  // Commnad 11: reset the count of encoder 1
  if (I2C_receiveBuffer[0] == 11)
    pulses1 = 0;
      
  // Commnad 12: reset the count of encoder 2
  else if (I2C_receiveBuffer[0] == 12)
    pulses2 = 0;
}

//--------------------------------------------------------------------//
void setup() 
{
  // I2C configuration as slave
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL); 
  Wire.begin(I2C_SLAVEADDR);
  Wire.setClock(100000); //100Kbps should be the same as arduino ros node
  Wire.onRequest(I2C_eventRequest);
  Wire.onReceive(I2C_eventReceive);
  I2C_receiveBuffer[0] = 99; // No commnad received
 
  Serial.begin(250000);
  pinMode(PIN_EN_A, INPUT);
  pinMode(PIN_EN_B, INPUT);
  pinMode(PIN_HALL, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_EN_A), encoder_count1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), encoder_count2, CHANGE);
}

void loop() 
{}

// END

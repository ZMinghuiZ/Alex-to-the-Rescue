#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <buffer.h>
#include <util/delay.h>
#include "Arduino.h"
#include "packet.h"
#include "constants.h"

typedef enum{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.
// counts per revolution
// R: 184, 180 -> 182
// L: 185, 180 -> 182

#define COUNTS_PER_REV      100.0 // After Calibration

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.5

//UART
#define BUFFER_LEN 256
volatile TBuffer Rbuf; //for receiving data
volatile TBuffer Sbuf; //for sending data

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  11  // Left forward pin
#define LR                  10   // Left reverse pin
#define RF                  5  // Right forward pin
#define RR                  6  // Right reverse pin

//Colour sensor
#define S0 7       
#define S1 8  
#define S2 9  
#define S3 12  
#define OUT 13  

int colour_data;
int RBG[3] = {0, 0, 0};        
int Black[3] = {330, 250, 330}; //baseline reading measured
int White[3] = {25, 20, 25};  //baseline reading measured

//Utrasonic sensor
#define TIMEOUT 2000
#define SOUND_SPEED 340
#define ULTRASONIC A1
#define ALARM A0
#define ECHO 4

volatile double distance;

// For turning
#define PI            3.141592654
// Alex's dimensions in cm
#define ALEX_LENGTH   100
#define ALEX_BREADTH    62.5
// Alex's diagonal - compute and stored once as it is expensive and doesn't rly change
float alexDiagonal = 0.0;
// Alex's turning circumference, calculated once
float alexCirc = 0.0;


/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

// Left and right encorder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Colourdetection status
volatile unsigned long colour;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whether we have moved a commanded dist
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //

  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  //params
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  statusPacket.params[10] = colour;
  statusPacket.params[11] = distance*10;
  statusPacket.params[12] = RBG[0];
  statusPacket.params[13] = RBG[1];
  statusPacket.params[14] = RBG[2];
  
  sendResponse(&statusPacket); 
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...){
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 

  DDRD = 0b00001100;       // pins 2/3 set to INPUT
  PORTD = 0b00001100;     // pins 2/3 HIGH
}


// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if (dir == FORWARD){
    leftForwardTicks++;

    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  
  if (dir == BACKWARD){
    leftReverseTicks++;

    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }

  if (dir == LEFT){
    leftReverseTicksTurns++;
  }

  if (dir == RIGHT){
    leftForwardTicksTurns++;
  }
}

void rightISR()
{
  if (dir == FORWARD){
    rightForwardTicks++;
  }
  
  if (dir == BACKWARD){
    rightReverseTicks++;
  }

  if (dir == LEFT){
    rightForwardTicksTurns++;
  }

  if (dir == RIGHT){
    rightReverseTicksTurns++;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.

  EICRA |= 0b00001010;     // INT0/1 both falling
  EIMSK |= 0b00000011;            // enable INT0/1
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect){
  leftISR();
}

ISR(INT1_vect){
  rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setBaud(unsigned long baudRate)
{
  unsigned int b;
  b = (unsigned int) round(16000000 / (16.0 * baudRate)) - 1;
  UBRR0H = (unsigned char) (b >> 8);
  UBRR0L = (unsigned char) b;
}

void setupSerial()
{
  setBaud(9600);
  UCSR0C = 0b00000110; // 8N1
  UCSR0A = 0;
  initBuffer(&Rbuf, BUFFER_LEN);
  initBuffer(&Sbuf, BUFFER_LEN);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We wilrl
// replace this later with bare-metal code.

void startSerial()
{
  UCSR0B = 0b10111000; // interrupt mode  
}

//Trigger when data is received
ISR(USART_RX_vect) 
{
  unsigned char data = UDR0;
  writeBuffer(&Rbuf, data);
}

//Triger when there is a data to be sent
ISR(USART_UDRE_vect) 
{
  unsigned char data;
  TBufferResult result = readBuffer(&Sbuf, &data);

  if (result == BUFFER_OK) {
    UDR0 = data;
  } else if (result == BUFFER_EMPTY) {
    UCSR0B &= 0b11011111;
  }
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;
  
  TBufferResult result;
  do
  {
    result = readBuffer(&Rbuf, &buffer[count]);
    if (result == BUFFER_OK)
    {
      count++;
    }
  } while (result == BUFFER_OK);
  /*while(Serial.available())
    buffer[count++] = Serial.read();*/

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  TBufferResult result = BUFFER_OK;

  int i;  
  
  for(i = 1; i < len; i++)
  {
    result = writeBuffer(&Sbuf, buffer[i]);
  }
  
  UDR0 = buffer[0]; // triger the interrupt  
  UCSR0B |= 0b00100000; // renable UDRE interupt
  
  /*Serial.write(buffer, len);*/
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
   DDRD |= 0b01100000;
   DDRB |= 0b00001100;
}

// Start the PWM for Alex's motors.
void startMotors()
{
  TCCR0B = 0b00000001;
  TCCR1B = 0b00000001;
  TCCR2B = 0b00000001;
  //no prescaler
}

void setupPWM()
{
  //timer 0 setup
  TCNT0 = 0;
  OCR0A = 0;
  OCR0B = 0;
  TCCR0A = 0b10100001;
  TIMSK0 |= 0b110;
  
  //timer 1 setup
  TCNT1 = 0;
  OCR1BH = 0;
  OCR1BL = 0;
  TCCR1A = 0b00100001;
  TIMSK1 |= 0b100;
  
  //timer 2 setup
  TCNT2 = 0;
  OCR2A = 0;
  TCCR2A = 0b100000001;
  TIMSK2 |= 0b010;
}

//motor direction and power control
void control(int direction_pin, int val)
{
  switch (direction_pin) {
     case LR:
      OCR1BH = 0;
      OCR1BL = val;
      break;
     case LF:
      OCR2A = val;
      break;
     case RR:
      OCR0A = val;
      break;
     case RF:
      OCR0B = val;
      break;
  }
}

//ultrasonic sensor
void setupProxyalarm()
{
  pinMode(ULTRASONIC, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(ALARM, OUTPUT);
}

void proxy_alarm()
{
  digitalWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
 
  long duration = pulseIn(ECHO, HIGH, TIMEOUT);
  
  if (duration > 0)
  {
    distance = duration / 2.0 / 1000000 * SOUND_SPEED * 100;//in centimeter
  }
  else
  {
    distance = 666.0;//out of range
  }
  delay(500);

  //ring alarm when distance is within 10cm

  if (distance <= 10.0)
  {
    ringalarm();
    delay(1000);
    stopalarm();
  }  
} 

void ringalarm()
{
  digitalWrite(ALARM, HIGH);
}

void stopalarm()
{
  digitalWrite(ALARM, LOW);
}

//coloursensor
void setupColour()
{
   pinMode(S0,OUTPUT);   
   pinMode(S1,OUTPUT);
   pinMode(S2,OUTPUT);
   pinMode(S3,OUTPUT);
   pinMode(OUT,INPUT);

   digitalWrite(S0,HIGH); 
   digitalWrite(S1,HIGH);
   //Putting S0/S1 on HIGH/HIGH levels means the output frequency scaling is at 100% 
}

void adjust(int i)
{
  if (i == 0)
  {
     digitalWrite(S2,LOW);        
     digitalWrite(S3,LOW);
     //RED
  }
  else if (i == 1)
  {
    digitalWrite(S2,LOW);
    digitalWrite(S3,HIGH);
    //BLUE
  }
  else
  {
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    //GREEN
  }
}

void identify()
{
   for (int i = 0; i < 3; i++)
   {
    adjust(i);
    colour_data = pulseIn(OUT,LOW);
    RBG[i] = map(colour_data, Black[i], White[i], 0, 255);
    delay(20);
   }

/*
 * colour = 0 means RED victim
 * colour = 1 means GREEN victim
 * colour = 2 means not a victim
 */
   if (RBG[0] > 200 && RBG[0] > RBG[2])
   {
     if(RBG[2] < 145)
     {
       colour = 0; //RED
     }
     else
     {
       colour = 2; //not victim
     }
   }
   else if (RBG[1] > 150)
   {
    colour = 2;
   }
   else if (RBG[2] > 135)
   {
     colour = 1; //GREEN
   }
   else
   {
    colour = 2;
   }
   
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  // code to tell us how far to move
  if (dist > 0){
    deltaDist = dist;
  } else {
    deltaDist = 999999;
  }

  newDist = forwardDist + deltaDist;
  
  dir = FORWARD;
  
  int val = pwmVal(speed);
  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  
  control(LF, val);
  control(RF, 1.2*val);
  control(LR, 0);
  control(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  // code to tell us how far to move
  if (dist > 0){
    deltaDist = dist;
  } else {
    deltaDist = 999999;
  }

  newDist = reverseDist + deltaDist;
  
  dir = BACKWARD;

  int val = pwmVal(speed);

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  control(LR, val);
  control(RR, 1.2*val);
  control(LF, 0);
  control(RF, 0);
}

// New function to estimate number of wheel ticks
// needed to turn an angle
unsigned long computeDeltaTicks(float ang){
  // We will assume that angular dist moved = lin dist moved in one wheel
  // revolution. This is (probably) incorrect but simplifies calculation.
  // # of wheel revs to make 1 full 360 turn is alexCirc/WHEEL_CIRC
  // This is for 360 deg. For ang deg it will be (ang*alexCirc)/(360*WHEEL_CIRC)
  // To convert to ticks, we multiply by COUNTS_PER_REV.

  unsigned long ticks = (unsigned long) ((ang*alexCirc*COUNTS_PER_REV)/(360.0*WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  
  int val = pwmVal(speed);

  if (ang == 0){
    deltaTicks = 9999999;
  } else {
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = leftReverseTicksTurns + deltaTicks;

  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  control(RR, 1.2*val);
  control(LF, val);
  control(RF, 0);
  control(LR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  
  int val = pwmVal(speed);

  if (ang == 0){
    deltaTicks = 9999999;
  } else {
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = rightReverseTicksTurns + deltaTicks;

  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  control(LR, val);
  control(RF, 1.2*val);
  control(LF, 0);
  control(RR, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  
  control(LF, 0);
  control(LR, 0);
  control(RF, 0);
  control(RR, 0);
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightForwardTicksTurns =0 ;
  rightReverseTicksTurns = 0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Alex's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
        break;
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
        break;
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
        break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
        break;
    case COMMAND_STOP:
        sendOK();
        stop();
        break;

    // telemetry commands
    case COMMAND_GET_STATS:
        sendOK();
        identify();//detect colour
        proxy_alarm();//detect distance 
        sendStatus();    
        break;
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
        break;
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // compute the diagonal
  alexDiagonal = sqrt((ALEX_LENGTH*ALEX_LENGTH)+(ALEX_BREADTH*ALEX_BREADTH));
  alexCirc = PI*alexDiagonal;
  
  // setup

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  setupPWM();
  setupProxyalarm();
  setupColour();
  startMotors();
  enablePullups();
  initializeState();
  sei();

  waitForHello(); // wait for handshake with pi
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 

  // controlling dist
  if (deltaDist > 0){
    if (dir == FORWARD){
      if (forwardDist >= newDist){
        deltaDist=0;
        newDist=0;
        stop();
      }
    } else if (dir == BACKWARD){
        if (reverseDist >= newDist){
          deltaDist=0;
          newDist=0;
          stop();
        }
    } else if (dir == STOP){
        deltaDist=0;
        newDist=0;
        stop();  
    }   
  }

  // controlling turning angle
  if (deltaTicks > 0){
    if (dir == LEFT){
      if (leftReverseTicksTurns >= targetTicks){
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == RIGHT){
      if (rightReverseTicksTurns >= targetTicks){
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == STOP){
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
      
}

/*
 * Project course - Arduino Assignment
 * Microcontroller familiarization
 * Arduino UNO Pin # Component
 * A0 Potentiometer
 * 2 Button 0
 * 3 Button 1
 * 9 Red LED channel
 * 10 Green LED channel
 * 11 Blue LED channel
 * 
 * 
 * References:
 * Timer library taken from this site:
 * https://playground.arduino.cc/Code/Timer1
 * 
 * Other websites referred:
 * https://www.arduino.cc/en/Reference/DigitalRead
 * 
 * 
 * 
 */

#include "TimerOne.h" 

/******************************* Start of Macro definitions**************************************/
#define BAUD 38400
#define PIN_BUTTON0 3
#define PIN_BUTTON1 2
#define POTENTIOMETER 0
#define RED_LED_CHANNEL 9
#define GREEN_LED_CHANNEL 10
#define BLUE_LED_CHANNEL 11

#define DEBOUNCE_DELAY  20000     //micro sec

/******************************* End of MACRO definitions********************************/

/******************************* Start of Type declarations*************************************/
typedef enum 
{
  STATE0=0,
  STATE1=1,
  STATE2=2
}STATE;

typedef enum
{
  C0=0,
  C1=1,
  C2=2,
  C3=3,
  C4=4,
  C5=5,
  C6=6,
  C7=7,  
}RGB_STATE;

/********************************* End of Type Declrations**************************************/

/*******************************Start of Global variable section *********************/
STATE state=STATE0;
RGB_STATE rgb_state=C0;


int statusLedPin = 13; // LED connected to digital pin 13
int a=0;

int potReadPin=POTENTIOMETER;
int rLedPin=RED_LED_CHANNEL;
int gLedPin=GREEN_LED_CHANNEL;
int bLedPin=BLUE_LED_CHANNEL;

int potValue=0;
float potVolt=0.0;
float potNorm=0.0;
float potNormInv=0.0;

int rVal=0,gVal=0,bVal=0;
float rVolt=0.0,rNorm=0.0,gVolt=0.0,gNorm=0.0,bVolt=0.0,bNorm=0.0;

int button1_pin = PIN_BUTTON1;
long button1_press_start_time=0;
int button1_press_time=0;
int button1_rise_detected=0;
int val_button1 = 0;     // variable to store the read value

int button0_pin = PIN_BUTTON0;
long button0_press_start_time=0;
int button0_press_time=0;
int button0_rise_detected=0;
int val_button0 = 0;     // variable to store the read value

long loop_last_time=0;
int loop_run_flag=0;

/*******************************End of golobal Variable section *****************************/


/*******************************Start of function section  **************************/


void change_state()
{
  switch(state)
  {
    case STATE0:
        state=STATE1;
        break;
    case STATE1:
        state=STATE2;
        break;
    case STATE2:
        state=STATE0;
        break;
  }

  Serial.print("State");
  Serial.print(state);
  Serial.print("\r");
}

void change_rgb_state()
{
  switch(rgb_state)
  {
    case C0:
        rgb_state=C1;
        digitalWrite(rLedPin, HIGH);
        digitalWrite(gLedPin, HIGH);
        digitalWrite(bLedPin, HIGH);
        break;
    case C1:
        rgb_state=C2;
        digitalWrite(rLedPin, HIGH);
        digitalWrite(gLedPin, HIGH);
        digitalWrite(bLedPin, LOW);
        break;
    case C2:
        rgb_state=C3;
        digitalWrite(rLedPin, HIGH);
        digitalWrite(gLedPin, LOW);
        digitalWrite(bLedPin, HIGH);
        break;
    case C3:
        rgb_state=C4;
        digitalWrite(rLedPin, HIGH);
        digitalWrite(gLedPin, LOW);
        digitalWrite(bLedPin, LOW);
        break;
    case C4:
        rgb_state=C5;
        digitalWrite(rLedPin, LOW);
        digitalWrite(gLedPin, HIGH);
        digitalWrite(bLedPin, HIGH);
        break;
    case C5:
        rgb_state=C6;
        digitalWrite(rLedPin, LOW);
        digitalWrite(gLedPin, HIGH);
        digitalWrite(bLedPin, LOW);
        break;
    case C6:
        rgb_state=C7;
        digitalWrite(rLedPin, LOW);
        digitalWrite(gLedPin, LOW);
        digitalWrite(bLedPin, HIGH);
        break;
    case C7:
        rgb_state=C0;
        digitalWrite(rLedPin, LOW);
        digitalWrite(gLedPin, LOW);
        digitalWrite(bLedPin, LOW);
        break;
  }

  rVal=analogRead(rLedPin);
  rVolt=(float)rVal*5.0/1023.0;
  rNorm=rVolt/5.0;
  gVal=analogRead(gLedPin);
  gVolt=(float)gVal*5.0/1023.0;
  gNorm=gVolt/5.0;
  bVal=analogRead(bLedPin);
  bVolt=(float)bVal*5.0/1023.0;
  bNorm=bVolt/5.0;

  Serial.print("RGB");
  Serial.print(rgb_state);
  Serial.print("\r");
}

void button0_pressed()
{
    //store the ti
    button0_press_start_time=Timer1.read();
    button0_rise_detected=1;
    Serial.print("*");
}

void button1_pressed()
{
    //store the ti
    button1_press_start_time=Timer1.read();
    button1_rise_detected=1;
    Serial.print("#");
}

void setup()
{
    //initialize serial port
    Serial.begin(BAUD); 

    //initialize input output pins
    pinMode(statusLedPin, OUTPUT);          // sets the digital pin 13 as output

    //RGB LED
    pinMode(rLedPin, OUTPUT);          // sets the digital pin 9 as output
    pinMode(gLedPin, OUTPUT);          // sets the digital pin 10 as output
    pinMode(bLedPin, OUTPUT);          // sets the digital pin 11 as output
    
    pinMode(button0_pin, INPUT);      // sets the digital pin 3 as input
    pinMode(button1_pin, INPUT);      // sets the digital pin 2 as input

    attachInterrupt(digitalPinToInterrupt(button0_pin), button0_pressed, RISING);
    attachInterrupt(digitalPinToInterrupt(button1_pin), button1_pressed, RISING);  

    //initialize timer
    Timer1.initialize();
    Timer1.stop();        //stop the counter
    Timer1.restart();     //set the clock to zero


    Serial.print("State");
    Serial.print(state);
    Serial.print("\r");
    Serial.print("RGB");
    Serial.print(rgb_state);
    Serial.print("\r");
}

void loop()
{
    //setting loop flag at 5Hz
    long time_now=Timer1.read();
    if((time_now-loop_last_time)>200000)
    {
      loop_run_flag=1;
      loop_last_time=time_now;
    }
    else if((time_now<loop_last_time) && ((1000000+time_now-loop_last_time)>200000))
    {
      loop_run_flag=1;
      loop_last_time=time_now;
    }

//Debugging purpose, testing loop rate
//    if(loop_run_flag==1)
//    {
//      loop_run_flag=0;
//      if(a==0)
//      {
//        a=1;
//        digitalWrite(statusLedPin, HIGH);
//      }
//      else
//      {
//        a=0;
//        digitalWrite(statusLedPin, LOW);
//      }
//    }

    val_button0 = digitalRead(button0_pin);   // read the input pin
    time_now=Timer1.read();
    if(val_button0==1)
    {
      if(button0_rise_detected==1)
      {
        if(time_now>button0_press_start_time)
        {
           button0_press_time=time_now-button0_press_start_time;
        }
        else
        {
           button0_press_time=1000000+time_now-button0_press_start_time;
        }
        
        if(button0_press_time>DEBOUNCE_DELAY)
        {
          change_state();          
          button0_press_time=0;
          button0_press_start_time=time_now;
          button0_rise_detected=0;
        } 
      }
    }

    val_button1 = digitalRead(button1_pin);   // read the input pin
    time_now=Timer1.read();
    if(val_button1==1)
    {
      if(button1_rise_detected==1)
      {
        if(time_now>button1_press_start_time)
        {
           button1_press_time=time_now-button1_press_start_time;
        }
        else
        {
           button1_press_time=1000000+time_now-button1_press_start_time;
        }
        
        if(button1_press_time>DEBOUNCE_DELAY)
        {
          if(state==STATE0)
          {
             change_rgb_state(); 
          }
          button1_press_time=0;
          button1_press_start_time=time_now;
          button1_rise_detected=0;
        } 
      }
    }

    delay(2);

    if(loop_run_flag)
    {
      if(state==STATE1)
      {
        potValue=analogRead(potReadPin);
        potVolt=(float)potValue*5.0/1023.0;
        potNorm=potVolt/5.0;

        analogWrite(rLedPin,(int)(255*rNorm*potNorm));
        analogWrite(gLedPin,(int)(255*gNorm*potNorm));
        analogWrite(bLedPin,(int)(255*bNorm*potNorm));        
 
        Serial.print("See\r");
        Serial.print(potValue);
        Serial.print("\r");
        Serial.print(potVolt);
        Serial.print("\r");
        Serial.print(potNorm);
        Serial.print("\r"); 
      }
      
      loop_run_flag=0;
      if(a==0)
      {
        a=1;
        digitalWrite(statusLedPin, HIGH);
      }
      else
      {
        a=0;
        digitalWrite(statusLedPin, LOW);
      }
    }
}

/***************************************End of Function section****************************/



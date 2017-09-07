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
#define PIN_BUTTON1 3
#define POTENTIOMETER A0
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

//Pushbutton related variables
typedef enum
{
  Pressed=0,
  released=1
}PUSHBUTTON;
/********************************* End of Type Declrations**************************************/

/*******************************Start of Global variable section *********************/
STATE state=STATE0;
PUSHBUTTON button1,button2;
int ledPin = 13; // LED connected to digital pin 13
int button0_pin = PIN_BUTTON0;
int val_button0 = 0;     // variable to store the read value

int button1_pin = PIN_BUTTON1;
int a=0;

long button0_press_start_time=0;
int button0_press_time=0;
int rise_detected=0;

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
}

void button0_pressed()
{
    //store the ti
    button0_press_start_time=Timer1.read();
    rise_detected=1;
    Serial.print("***");
}

void setup()
{
    //initialize serial port
    Serial.begin(BAUD); 

    //initialize input output pins
    pinMode(ledPin, OUTPUT);          // sets the digital pin 13 as output
    pinMode(button0_pin, INPUT);      // sets the digital pin 7 as input

    attachInterrupt(digitalPinToInterrupt(button0_pin), button0_pressed, RISING);  

    //initialize timer
    Timer1.initialize();
    Timer1.stop();        //stop the counter
    Timer1.restart();     //set the clock to zero
}

void loop()
{
    //setting loop flag at 50Hz
    long time_now=Timer1.read();
    if((time_now-loop_last_time)>100000)
    {
      loop_run_flag=1;
      loop_last_time=time_now;
    }
    else if((time_now<loop_last_time) && ((1000000+time_now-loop_last_time)>100000))
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
//        digitalWrite(ledPin, HIGH);
//      }
//      else
//      {
//        a=0;
//        digitalWrite(ledPin, LOW);
//      }
//    }

    val_button0 = digitalRead(button1_pin);   // read the input pin
    time_now=Timer1.read();
    if(val_button0==1)
    {
      if(rise_detected==1)
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
          rise_detected=0;
        } 
      }
    }

    delay(2);

    if(loop_run_flag)
    {
      loop_run_flag=0;
      Serial.print("State");
      Serial.print(state);
      Serial.print("\r");  

      if(a==0)
      {
        a=1;
        digitalWrite(ledPin, HIGH);
      }
      else
      {
        a=0;
        digitalWrite(ledPin, LOW);
      }
    }
}

/***************************************End of Function section****************************/



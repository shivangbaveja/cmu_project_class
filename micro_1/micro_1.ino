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
#define BAUD 9600
#define PIN_BUTTON0 2
#define PIN_BUTTON1 3
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
RGB_STATE rgb_state=C7;


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

// Serial data
#define MAX_BUFF_LEN 20
char data_buffer[MAX_BUFF_LEN];
char data[4];
int index=0;
int read_index=0;
int data_index=0;
int rx_count=0;
int data_ready=0;
int command_start=0;

String str,str2;
int start_parse=0;

int test_flag=0;


/*
 * Ultrasonic variables
 */
 const int ultraPin = 7;
 long ultraPulse, ultraInches, ultraCM;
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
        analogWrite(rLedPin,255);
        analogWrite(gLedPin,255);
        analogWrite(bLedPin,255);
        rNorm=0;
        gNorm=0;
        bNorm=0;
        break;
    case C1:
        rgb_state=C2;
        analogWrite(rLedPin,255);
        analogWrite(gLedPin,255);
        analogWrite(bLedPin,0);
        rNorm=0;
        gNorm=0;
        bNorm=1;
        break;
    case C2:
        rgb_state=C3;
        analogWrite(rLedPin,255);
        analogWrite(gLedPin,0);
        analogWrite(bLedPin,255);
        rNorm=0;
        gNorm=1;
        bNorm=0;
        break;
    case C3:
        rgb_state=C4;
        analogWrite(rLedPin,255);
        analogWrite(gLedPin,0);
        analogWrite(bLedPin,0);
        rNorm=0;
        gNorm=1;
        bNorm=1;
        break;
    case C4:
        rgb_state=C5;
        analogWrite(rLedPin,0);
        analogWrite(gLedPin,255);
        analogWrite(bLedPin,255);
        rNorm=1;
        gNorm=0;
        bNorm=0;
        break;
    case C5:
        rgb_state=C6;
        analogWrite(rLedPin,0);
        analogWrite(gLedPin,255);
        analogWrite(bLedPin,0);
        rNorm=1;
        gNorm=0;
        bNorm=1;
        break;
    case C6:
        rgb_state=C7;
        analogWrite(rLedPin,0);
        analogWrite(gLedPin,0);
        analogWrite(bLedPin,255);
        rNorm=1;
        gNorm=1;
        bNorm=0;
        break;
    case C7:
        rgb_state=C0;
        analogWrite(rLedPin,0);
        analogWrite(gLedPin,0);
        analogWrite(bLedPin,0);
        rNorm=1;
        gNorm=1;
        bNorm=1;
        break;
  }

  Serial.print("RGB");
  Serial.print(rgb_state);
  Serial.print("\r");
}

void button0_pressed()
{
    //store the ti
    button0_press_start_time=Timer1.read();
    button0_rise_detected=1;
//    Serial.print("*");
}

void button1_pressed()
{
    //store the ti
    button1_press_start_time=Timer1.read();
    button1_rise_detected=1;
//    Serial.print("#");
}

void setup()
{
    /*
     * Ultrasonic sensor related stuff
     */
     pinMode(ultraPin, INPUT);
  
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


    //default values RGB state C7
    analogWrite(rLedPin,0);
    analogWrite(gLedPin,0);
    analogWrite(bLedPin,0);  
    rNorm=1;
    gNorm=0;
    bNorm=1;

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

      while(Serial.available()) 
      {
             str= Serial.readString();// read the incoming data as string
             start_parse=1;
             char ch=str[0];
             if(ch=='r' || ch=='g' || ch=='b' || ch=='R' || ch=='G' || ch=='B')
             {
                str2=str;
                str2.replace(ch,'0');
                int val=str2.toInt();
                
                int test_bad=0;
                int i=1;
                while(str2.charAt(i)!='\0')
                {
                    if(str2.charAt(i)<48 || str2.charAt(i)>57)
                    {
                      test_bad=1;
                      break;
                    }
                    i++;
                }

                 if(state==STATE2 && val>=0 && val<=255 && str.length()<=4 && test_bad==0)
                  {            
                    int out=255-val;
                    switch(ch)
                    {
                      case 'r':
                      analogWrite(rLedPin,out);
                      break;
                      case 'R':
                      analogWrite(rLedPin,out);
                      break;
                      case 'g':
                      analogWrite(gLedPin,out);
                      break;
                      case 'G':
                      analogWrite(gLedPin,out);
                      break;
                      case 'b':
                      analogWrite(bLedPin,out);
                      break;
                      case 'B':
                      analogWrite(bLedPin,out);
                      break;
                    }
                    Serial.print(ch);
                    Serial.print(val);
                    Serial.print("\r");
                  }
              }
         }  

    if(loop_run_flag)
    {
      if(state==STATE1)
      {
        potValue=analogRead(potReadPin);
        potVolt=(float)potValue*5.0/1023.0;
        potNorm=potVolt/5.0;
        potNormInv=1-potNorm;

        int out1,out2,out3;
        out1=255 - (int)(255*rNorm*potNormInv);
        out2=255 - (int)(255*gNorm*potNormInv);
        out3=255 - (int)(255*bNorm*potNormInv);
        
        analogWrite(rLedPin,out1);
        analogWrite(gLedPin,out2);
        analogWrite(bLedPin,out3);        
      }

      /*
       * Get and print ultrasonic data
       */
      ultraPulse = pulseIn(ultraPin, HIGH);
      
      //147uS per inch
      ultraInches = ultraPulse / 147;
      
      //change Inches to centimetres
      ultraCM = ultraInches * 2.54;
      Serial.print(ultraCM);
      Serial.print("cm");
      Serial.println();
      /*********************************/
      

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



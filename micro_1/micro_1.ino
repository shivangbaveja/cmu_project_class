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
//#include "PinChangeInt.h"

/******************************* Start of Macro definitions**************************************/
#define BAUD 9600
#define PIN_BUTTON0 (2)
#define ENCODER1    (3)
#define POTENTIOMETER (0)
#define ULTRASONIC_PIN  (8)

//anticlockwise when looking at shaft from the front, when high given to F and low given to B
#define M_CTRL_F  (7)       //Motor control forward, goes to L1, 
#define M_CTRL_B (5)        //Motor control backward, goes to L2
#define ENCODER2  (4)
#define SPEED_CONTROL_PIN   (6)

#define P_GAIN      (5.0)
#define I_GAIN      (0.0)
#define D_GAIN      (1.0)
#define INT_MAX     (150.0)

#define DEBOUNCE_DELAY  20000     //micro sec

/******************************* End of MACRO definitions********************************/

/******************************* Start of Type declarations*************************************/
typedef enum 
{
  STATE0=0,
  STATE1=1,
  STATE2=2
}STATE;

/********************************* End of Type Declrations**************************************/

/*******************************Start of Global variable section *********************/
STATE state=STATE0;

int statusLedPin = 13; // LED connected to digital pin 13
int a=0;

int potReadPin=POTENTIOMETER;

int potValue=0;
float potVolt=0.0;
float potNorm=0.0;
float potNormInv=0.0;

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
int hz1_loop=0;
int hz5_loop=0;

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
 * Encoder specific variables
 */
volatile long encoder_count=0;

/*
 * Ultrasonic variables
 */
 const int ultraPin = ULTRASONIC_PIN;
 long ultraPulse, ultraInches, ultraCM;
/*******************************End of golobal Variable section *****************************/

/*
 * PID control
 */
float error_pos=0;
int target_pos=0;
int current_pos=0;
float integrator=0;
long int pid_old_time=0;
float control_out=0;
 /******************************/


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

void button0_pressed()
{
    button0_press_start_time=Timer1.read();
    button0_rise_detected=1;
}

void encoder1_change()
{
    bool encoder1_val=PIND & B00001000;
    bool encoder2_val=PIND & B00010000;
    if(encoder1_val==encoder2_val)
    {
      encoder_count--;
    }
    else
    {
      encoder_count++;
    }
}

void run_motor()
{
  if(control_out>255)
  {
    control_out=255;
  }
  else if(control_out<-255)
  {
    control_out=-255;
  }
  
  if(control_out<0)
  {
    //go reverse
    digitalWrite(M_CTRL_B, HIGH);
    digitalWrite(M_CTRL_F, LOW);
    analogWrite(SPEED_CONTROL_PIN,abs(int(control_out)));
    
    Serial.print("\nError:");
    Serial.print(error_pos);
    Serial.print("\nBackward:");
    Serial.print(control_out);
    Serial.print("\nIntegrator:");
    Serial.print(integrator);
    Serial.println();
  }
  else
  {
    //go forward
    digitalWrite(M_CTRL_F, HIGH);
    digitalWrite(M_CTRL_B, LOW);
    analogWrite(SPEED_CONTROL_PIN,abs(int(control_out)));
    
    Serial.print("\nError:");
    Serial.print(error_pos);
    Serial.print("\nforward:");
    Serial.print(control_out);
    Serial.print("\nIntegrator:");
    Serial.print(integrator);
    Serial.println();
  }
}

void setup()
{
    /*
     * Ultrasonic sensor related stuff
     */
     pinMode(ultraPin, INPUT);

     /*
      * Motor initializations
      */
    pinMode(M_CTRL_B, OUTPUT);
    pinMode(M_CTRL_F, OUTPUT);
    pinMode(SPEED_CONTROL_PIN, OUTPUT);
      
    //initialize serial port
    Serial.begin(BAUD); 

    //initialize input output pins
    pinMode(statusLedPin, OUTPUT);          // sets the digital pin 13 as output
    
    pinMode(button0_pin, INPUT);      // sets the digital pin 2 as input
    pinMode(ENCODER1, INPUT);      // sets the digital pin 3 as input
    pinMode(ENCODER2, INPUT);      // sets the digital pin 2 as input

    attachInterrupt(digitalPinToInterrupt(button0_pin), button0_pressed, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER1), encoder1_change, CHANGE);  

    //initialize timer
    Timer1.initialize();
    Timer1.stop();        //stop the counter
    Timer1.restart();     //set the clock to zero

    Serial.print("State");
    Serial.print(state);
    Serial.print("\r");
}

void loop()
{
    //setting loop flag at 5Hz
    long time_now=Timer1.read();
    if((time_now-loop_last_time)>20000)
    {
      loop_run_flag=1;
      loop_last_time=time_now;
    }
    else if((time_now<loop_last_time) && ((1000000+time_now-loop_last_time)>20000))
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

    delay(2);

    while(Serial.available()) 
    {
         str= Serial.readString();// read the incoming data as string
         start_parse=1;
         char ch=str[0];
         if(ch=='d' || ch=='D')
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

             if(val>=0 && val<=360 && str.length()<=4 && test_bad==0)
            {            
              int out=val;
              switch(ch)
              {
                case 'd':
                Serial.print("\rReceived target:");
                Serial.print(out);
                target_pos=out;
                break;
                case 'D':
                Serial.print("\rReceived target:");
                Serial.print(out);
                target_pos=out;
                break;
              }
            }
          }
    }  

    if(loop_run_flag)
    {
      //clear the flag
      loop_run_flag=0;

      //Blink the status LED
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
      
      /*
       * Potentiometer read
       */
        potValue=analogRead(potReadPin);
        potVolt=(float)potValue*5.0/1023.0;
        potNorm=potVolt/5.0;
        potNormInv=1-potNorm; 
      /*********************************/     

      /*
       * Get and print ultrasonic data
       */
      ultraPulse = pulseIn(ultraPin, HIGH);
      
      //147uS per inch
      ultraInches = ultraPulse / 147;
      
      //change Inches to centimetres
      ultraCM = ultraInches * 2.54;
//      Serial.print(ultraCM);
//      Serial.print("cm");
//      Serial.println();
      /*********************************/

      /*
       * PID control
       */
      long int time_now=millis();
      float dt=(time_now-pid_old_time)/1000.0;
      pid_old_time=time_now;
      if(dt<0.01)
      {
        dt=0.01;
      }
      else if(dt>1)
      {
        dt=1;
      }

      current_pos=encoder_count;
      error_pos=target_pos-current_pos;

      if(control_out<255 && error_pos>0)
      {
        integrator=integrator+error_pos*dt;
      }
      else if(control_out>-255 && error_pos<0)
      {
        integrator=integrator+error_pos*dt;
      }
      
      if(integrator>INT_MAX)
      {
        integrator=INT_MAX;
      }
      else if(integrator<-INT_MAX)
      {
        integrator=-INT_MAX;
      }

      control_out=P_GAIN*error_pos + I_GAIN*integrator;
      run_motor();




      
      //5Hz loop
      hz5_loop++;
      if(hz5_loop>10)
      {
        hz5_loop=0;
      }
      
      /*
       * Slow loop 1Hz
       */
      static int motor_dir=1;
      if(hz1_loop>=50)
      {
        hz1_loop=0;
      }
      else
      {
        hz1_loop++;
      }
        /**************************/
    }
}


/***************************************End of Function section****************************/



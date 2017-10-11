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
#include <Servo.h>
#include <medianFilter.h>


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

#define SPEED_I_GAIN    (0.7)
#define SPEED_P_GAIN    (2.0)
#define SPEED_INT_MAX   (150)

#define DEBOUNCE_DELAY  20000     //micro sec

/******************************* End of MACRO definitions********************************/

/******************************* Start of Type declarations*************************************/
typedef enum 
{
  STATE0=0,
  STATE1=1,
  STATE2=2,
  STATE3=3,
  STATE4=4
}STATE;

/********************************* End of Type Declrations**************************************/

/*******************************Start of Global variable section *********************/

/*
 * Hari's servo IR integration
 */
const int ir=A1;
const int servo=9;
float IRinput[25];
float IRdistance;
float IRvolt,IRvolt1;
float tmppos;
int pos;
Servo myservo; 
 /**************************/

STATE state=STATE0;

int statusLedPin = 13; // LED connected to digital pin 13
int a=0;

int potReadPin=POTENTIOMETER;

int potValue=0;
float potVolt=0.0;
float potNorm=0.0;

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
long last_encoder_count=0;

/*
 * Ultrasonic variables
 */
 const int ultraPin = ULTRASONIC_PIN;
 long ultraPulse, ultraInches;
long ultraCM;
medianFilter Filter;
int filtered_ultra=0;
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

float speed_target=0;
float speed_actual=0;
float speed_control_out=0;
float speed_integrator=0;
float speed_error=0;
float speed_I=SPEED_I_GAIN;
float speed_P=SPEED_P_GAIN;
int speed_dir=1;


float p_gain=P_GAIN;
float i_gain=I_GAIN;
float d_gain=D_GAIN;
float int_max=INT_MAX;
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
        state=STATE3;
        break;
    case STATE3:
        state=STATE4;
        break;
    case STATE4:
        state=STATE1;
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

void stop_motor()
{
  analogWrite(SPEED_CONTROL_PIN,0);
}

void motor_forward_pulse(int pulse)
{
  if(pulse<0)
  {
    pulse = 0;
  }
  else if(pulse>255)
  {
    pulse = 255;
  }
  analogWrite(SPEED_CONTROL_PIN,pulse);
  digitalWrite(M_CTRL_F, HIGH);
  digitalWrite(M_CTRL_B, LOW);
}

void motor_backward_pulse(int pulse)
{
  if(pulse<0)
  {
    pulse = 0;
  }
  else if(pulse>255)
  {
    pulse = 255;
  }
  analogWrite(SPEED_CONTROL_PIN,pulse);
  digitalWrite(M_CTRL_F, LOW);
  digitalWrite(M_CTRL_B, HIGH);
}

void run_motor_speed()
{
   if(speed_control_out>255)
   {
    speed_control_out=255;
   }
   else if(speed_control_out<-255)
   {
    speed_control_out=-255;
   }
  
  //go forward
  if(speed_target>0)
  {
    float control_out=(255 + speed_control_out)/2;
    digitalWrite(M_CTRL_F, HIGH);
    digitalWrite(M_CTRL_B, LOW);
    analogWrite(SPEED_CONTROL_PIN,abs(int(control_out))); 
  }
  //go backward
  else
  {
    float control_out=(255 - speed_control_out)/2;
    digitalWrite(M_CTRL_F, LOW);
    digitalWrite(M_CTRL_B, HIGH);
    analogWrite(SPEED_CONTROL_PIN,abs(int(control_out))); 
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
    
//    Serial.print("\nError:");
//    Serial.print(error_pos);
//    Serial.print("\nBackward:");
//    Serial.print(control_out);
//    Serial.print("\nIntegrator:");
//    Serial.print(integrator);
//    Serial.println();
  }
  else
  {
    //go forward
    digitalWrite(M_CTRL_F, HIGH);
    digitalWrite(M_CTRL_B, LOW);
    analogWrite(SPEED_CONTROL_PIN,abs(int(control_out)));
    
//    Serial.print("\nError:");
//    Serial.print(error_pos);
//    Serial.print("\nforward:");
//    Serial.print(control_out);
//    Serial.print("\nIntegrator:");
//    Serial.print(integrator);
//    Serial.println();
  }
}

/*
 * Hari's code
 */
void sort(float a[]) 
{
    for(int i=0; i<25; i++) 
    {
        bool flag = true;
        for(int j=0; j<(25-(i+1)); j++) 
        {
            if(a[j] > a[j+1])
            {
                int t = a[j];
                a[j] = a[j+1];
                a[j+1] = t;
                flag = false;
            }
        }
        if (flag) 
        break;
    }
    //Serial.println(1);
}
/*********************/

void setup()
{
    Filter.begin();
    /*
     * Hari's code
     */
     // put your setup code here, to run once:
      myservo.attach(9);
      myservo.write(0);
      pinMode(ir, INPUT);
      /****************************/
  
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

//    delay(5);

    while(Serial.available()) 
    {
         str= Serial.readString();// read the incoming data as string
         start_parse=1;
         char ch=str[0];
         if(ch=='d' || ch=='D' || ch=='p' || ch=='i' || ch=='s')
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
                Serial.print("\nReceived target:");
                Serial.print(out);
                target_pos=out;
                break;
                case 'D':
                Serial.print("\nReceived target:");
                Serial.println(out);
                target_pos=out;
                break;
                case 'p':                
                p_gain=(float)out;
                Serial.print("\nPgain:");
                Serial.print(p_gain);
                break;
                case 'i':
                i_gain=(float)out;
                Serial.print("\nIgain:");
                Serial.println(i_gain);
                break;
                case 'g':
                d_gain=(float)out;
                Serial.print("\nDgain:");
                Serial.print(out);
                break;
                case 's':
                speed_target=(float)out;
                Serial.print("\nST:");
                Serial.print(speed_target);
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

      /**********************************/
      /*
       * Get and print ultrasonic data
       */
      ultraPulse = pulseIn(ultraPin, HIGH);
      
      //147uS per inch
      ultraInches = ultraPulse / 147;
      //change Inches to centimetres and add a low pass filter
      ultraCM = ultraInches * 2.54;

      if(ultraCM>20)
      {
        filtered_ultra= Filter.run(ultraCM);
      }
      
//      Serial.print(ultraCM);
//      Serial.print("ucm");
//      Serial.println();
//      Serial.print(filtered_ultra);
//      Serial.print("fcm");
//      Serial.println();
      /**********************************/

     
      /*********************************/

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

      switch(state)
      {
        //Motors off
        case STATE0:
            
            //SHUT MOTORS
            stop_motor();

            //TODO: Add code for stopping servo and stepper motor

            //clear some variables
            encoder_count=0;
            target_pos=0;
            integrator=0;
            last_encoder_count=0;
            speed_integrator=0;
            speed_target=0;
            break;

        //DC motor pos control + ultrasonic sensor
        case STATE1:
            /*
             * PID control
             */

            target_pos=map(filtered_ultra,20,200,0,360);
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
      
            control_out=p_gain*error_pos + i_gain*integrator;
            run_motor();

            speed_integrator=0;
            speed_target=0;
            break;

        //DC motor speed control + potentiometer
        case STATE2:
            /*
             * Potentiometer read
             */
              potValue=analogRead(potReadPin);
              potVolt=(float)potValue*5.0/1023.0;
              potNorm=potVolt/5.0;

//              Serial.print("\rPot:");
//              Serial.println(potNorm);
              speed_target=100.0 + 200.0*potNorm;
            /*********************************/

            //degree per sec
            speed_actual=(encoder_count-last_encoder_count)/dt;
            last_encoder_count=encoder_count;

//            Serial.print("\rSpeed_actual:");
//            Serial.println(speed_actual);
//            Serial.print("Speed_target:");
//            Serial.println(speed_target);

            if(speed_target==0)
            {
              //SHUT MOTORS
               stop_motor();
               speed_integrator=0;
               speed_control_out=0;
            }
            else
            {
              speed_error = speed_target-speed_actual;
              
              if(speed_control_out<255 && speed_error>0)
              {
                speed_integrator = speed_integrator + speed_error*dt;
              }
              else if(speed_control_out>-255 && speed_error<0)
              {
                speed_integrator = speed_integrator + speed_error*dt;
              }
              
              if(speed_integrator>SPEED_INT_MAX)
              {
                speed_integrator=SPEED_INT_MAX;
              }
              else if(speed_integrator<-SPEED_INT_MAX)
              {
                speed_integrator=-SPEED_INT_MAX;
              }
              
              speed_control_out = speed_integrator*speed_I + speed_error*speed_P;
              run_motor_speed(); 
            }

            target_pos=0;
            integrator=0;
            break;

        //Servo motor + IR distance sensor
        case STATE3:

          // put your main code here, to run repeatedly:
  
          for (int i=0; i<25; i++)
          {
              // Read analog value
              IRinput[i] = analogRead(ir);
          }   
          sort(IRinput);
          
          IRvolt = map((IRinput[13]+IRinput[12])/2.0,0,1023,0,5000);
          IRvolt1 = IRvolt/1000.0;
          
          if(IRvolt1 >=0.85 && IRvolt1<=2.5)
          {
            IRdistance = 23.4 * IRvolt1 * IRvolt1 -115.7*IRvolt1 + 156.2; // from transfer function
            tmppos = map(IRdistance,40.0,70.0,0.0,18.0)*10;
            //pos = (int) tmppos;
            Serial.print(IRdistance);
            Serial.println(tmppos);
            myservo.write(tmppos);
            if (tmppos==172.0)                                   
            myservo.write(180.0);
            //range is 0.85v to 2.5v
          //distance range is 13.24cm to 74.75cm
          //working range is 40 to 71cm
                         
          //delay for servo in site is 15ms
            }
            else
            {
              IRdistance=-100;
            }




            //SHUT MOTORS
            stop_motor();

            //TODO: Add code for stopping servo and stepper motor

            //clear some variables
            encoder_count=0;
            target_pos=0;
            integrator=0;
            last_encoder_count=0;
            speed_integrator=0;
            speed_target=0;
           
            break;

        //Stepper motor + force sensor
        case STATE4:
            
            break;
      }
      
      
//      //5Hz loop
//      hz5_loop++;
//      if(hz5_loop>10)
//      {
//        hz5_loop=0;
//      }
//      
//      /*
//       * Slow loop 1Hz
//       */
//      static int motor_dir=1;
//      if(hz1_loop>=50)
//      {
//        hz1_loop=0;
//      }
//      else
//      {
//        hz1_loop++;
//      }
        /**************************/
    }
}


/***************************************End of Function section****************************/



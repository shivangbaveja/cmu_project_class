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
 * 
 * Other websites referred:
 * https://www.arduino.cc/en/Reference/DigitalRead
 * 
 * 
 * 
 */

//#include "PinChangeInt.h"
#include <Servo.h>
#include <medianFilter.h>
#include <Stepper.h>


//Nick's code
#define DIR 11
#define STEP 10


/******************************* Start of Macro definitions**************************************/
#define BAUD 38400
#define PIN_BUTTON0 (2)
#define ENCODER1    (3)
#define POTENTIOMETER (0)
#define ULTRASONIC_PIN  (8)
#define SERVO           (9)

//anticlockwise when looking at shaft from the front, when high given to F and low given to B
#define M_CTRL_F  (5)       //Motor control forward, goes to L1, 
#define M_CTRL_B (7)        //Motor control backward, goes to L2
#define ENCODER2  (4)
#define SPEED_CONTROL_PIN   (6)

#define P_GAIN      (2.0)
#define I_GAIN      (0.0)
#define D_GAIN      (0.0)
#define INT_MAX     (150.0)

#define SPEED_I_GAIN    (0.7)
#define SPEED_P_GAIN    (2.0)
#define SPEED_INT_MAX   (150)

#define DEBOUNCE_DELAY  20     //micro sec

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
//float IRinput[25];
float IRdistance;
float IRvolt,IRvolt1;
float tmppos;
int pos;
Servo myservo; 
int servo_target=0;
float filtered_IR=0;
 /**************************/

/*
 * Nick's code
 */
 int fsrPin = A2;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
unsigned long fsrConductance; 
long fsrForce;       // Finally, the resistance converted to force

bool stepper_at_pos = false;
float desired_angle = 0.0;
float desired_step = 0.0;
int step_size = 0;

const int stepsPerRevolution = 800;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, DIR, STEP);
 /*************************/

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
medianFilter Filter,Filter_IR;
int filtered_ultra=0;
/*******************************End of golobal Variable section *****************************/

/*
 * PID control
 */
int control_input=0;      //0 sensor controlled, 1 user controlled
 
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
  control_input=0;
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
  Serial.println(state);
}

void button0_pressed()
{
    button0_press_start_time=millis();
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

int i=0;
int dir=1;

void run_motor_sweep()
{
  
  if(i>10)
  {
    i=0;
    if(dir==1)
    {
      dir=-1;
      target_pos=0;
      motor_forward_pulse(250);
      Serial.print(i);
      Serial.println("Forward");
    }
    else
    {
      dir=1;
      target_pos=100;
      Serial.print(i);
      motor_backward_pulse(250);
      Serial.println("Backward");
    }
  }
  else
  {
    i++;
  }
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
//    Serial.print("\nTargetpos:");
//    Serial.print(target_pos);
//    Serial.print("\ncurrentpos:");
//    Serial.print(current_pos);
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
//    Serial.print("\nTargetpos:");
//    Serial.print(target_pos);
//    Serial.print("\ncurrentpos:");
//    Serial.print(current_pos);
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

void  send_telemetry()
{
  int M=0,D=0,S=0;
  switch(state)
  {
    case STATE0:
        M=0;
        D=0;
        S=0;
        break;
    case STATE1:
        M=current_pos;
        D=filtered_ultra;
        S=1;
        break;
    case STATE2:
        M=speed_actual;
        D=(int)(100.0*potVolt);
        S=2;
        break;
    case STATE3:
        M=servo_target;
        D=IRdistance;
        S=3;
        break;
    case STATE4:
        M=desired_angle;
        D=fsrForce;
        S=4;
        break;
  }

  if(M>=500)
  {
    M=500;
  }
  else if(M<0)
  {
    M=0;
  }

  if(D>=500)
  {
    D=500;
  }
  else if(D<0)
  {
    D=0;
  }
  
  String string1 = "S";                                     // using a constant String
  String string2 =  String(S);
  String string3 = "M";                                     // using a constant String
  String string4 =  String(M);
  String string5 = "D";                                     // using a constant String
  String string6 =  String(D);  
  String string7 =  "#";  

  String string=string1 + string2 + string3 + string4 + string5 + string6 + string7;
  Serial.println(string);
}

void setup()
{
    Filter.begin();
    Filter_IR.begin();

    /*
     * Nick's code
     */
    myStepper.setSpeed(60);
    /**********************/
    
    /*
     * Hari's code
     */
     // put your setup code here, to run once:
     // put your setup code here, to run once:
      myservo.attach(SERVO);
      //myservo.write(0);
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

    Serial.print("State");
    Serial.print(state);
    Serial.println("\r");
}

void loop()
{
    //setting loop flag at 5Hz
    long time_now=millis();
    if((time_now-loop_last_time)>20)
    {
      loop_run_flag=1;
      loop_last_time=time_now;
    }

    bool val_button0=digitalRead(button0_pin);   // read the input pin
    time_now=millis();
    if(val_button0==true)
    {
      if(button0_rise_detected==1)
      {
        if(time_now>button0_press_start_time)
        {
           button0_press_time=time_now-button0_press_start_time;
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

    while(Serial.available()) 
    {
         str= Serial.readString();// read the incoming data as string
         start_parse=1;
         char ch=str[0];
         if(ch=='d' || ch=='D' || ch=='p' || ch=='i' || ch=='s' || ch=='x' || ch=='X' || ch=='S')
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
                case 'D':
                Serial.print("\nReceived target:");
                Serial.println(out);
                 if(state==STATE1)
                {
                  target_pos=out;
                  control_input=1;
                }
                else if(state==STATE2)
                {
                  
                }
                else if(state==STATE3)
                {
                  servo_target=out;
                  control_input=1;
                }
                else if(state==STATE4)
                {
                  desired_angle=out;
                  control_input=1;
                }
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
                case 'S':
                
                Serial.print("\nST:");
                Serial.print(speed_target);
                
                 if(state==STATE1)
                {
                }
                else if(state==STATE2)
                {
                  speed_target=(float)out;
                  control_input=1;
                }
                else if(state==STATE3)
                {                  
                }
                break;
                case 'x':
                control_input=0;
                break;
                case 'X':
                control_input=0;
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
//      Serial.print("fcm");
//      Serial.println(filtered_ultra);
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

      //degree per sec
      speed_actual=(encoder_count-last_encoder_count)/dt;
      last_encoder_count=encoder_count;
//      Serial.print("Speed:");
//      Serial.println(speed_actual);

      send_telemetry();

      switch(state)
      {
        //Motors off
        case STATE0:
            
            //SHUT MOTORS
            stop_motor();

            //TODO: Add code for stopping servo and stepper motor

            //clear some variables
            encoder_count=0;
            last_encoder_count=0;
            target_pos=0;
            integrator=0;
            speed_integrator=0;
            speed_target=0;
            break;

        //DC motor pos control + ultrasonic sensor
        case STATE1:
            /*
             * PID control
             */
            if(control_input==0)
            {
              target_pos=map(filtered_ultra,20,200,0,360);
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

            if(i_gain>0.1)
            {
              if(integrator*i_gain>INT_MAX)
              {
                integrator=INT_MAX/i_gain;
              }
              else if(integrator*i_gain<-INT_MAX)
              {
                integrator=-INT_MAX/i_gain;
              } 
            }
      
            control_out=p_gain*error_pos + i_gain*integrator;
            run_motor();

            speed_integrator=0;
            speed_target=0;
//            Serial.print("current_pos:");
//            Serial.println(current_pos);
//            Serial.print("control_out:");
//            Serial.println(control_out);
            break;

        //DC motor speed control + potentiometer
        case STATE2:
            /*
             * Potentiometer read
             */
              potValue=analogRead(potReadPin);
              potVolt=(float)potValue*5.0/1023.0;
              potNorm=potVolt/5.0;

//            Serial.print("\rPot:");
//            Serial.println(potNorm);
              if(control_input==0)
              {
                speed_target=100.0 + 300.0*potNorm;
              }
            /*********************************/

        

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
  
//          for (int i=0; i<25; i++)
//          {
//              // Read analog value
//              IRinput[i] = analogRead(ir);
//          }   
//          sort(IRinput);

          filtered_IR= Filter_IR.run(analogRead(ir));
          
          IRvolt = map(filtered_IR,0,1023,0,5000);
          IRvolt1 = IRvolt/1000.0;
          IRdistance = 23.4 * IRvolt1 * IRvolt1 -115.7*IRvolt1 + 156.2; // from transfer function
          tmppos = map(IRdistance,40.0,70.0,0.0,18.0)*10;

          if(control_input==0)
          {
            if(IRvolt1 >=0.85 && IRvolt1<=2.5)
            {
                servo_target=tmppos;
//                Serial.print(IRdistance);
//                Serial.println(servo_target);
                myservo.write(servo_target);
              //range is 0.85v to 2.5v
              //distance range is 13.24cm to 74.75cm
              //working range is 40 to 71cm
                           
            //delay for servo in site is 15ms
            }
            else
            {
              IRdistance=-100;
            }
          }
          else
          {
//            Serial.print("\nhere");
            myservo.write(servo_target);
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

            digitalWrite(DIR, HIGH);

          fsrReading = analogRead(fsrPin); 
          
          // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
          fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
          
          // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
          // so FSR = ((Vcc - V) * R) / V        yay math!
          fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
          fsrResistance *= 10000;                // 10K resistor
          fsrResistance /= fsrVoltage;
          fsrConductance = 1000000;           // we measure in micromhos so 
          fsrConductance /= fsrResistance; 
          
          // Use the two FSR guide graphs to approximate the force
          if (fsrConductance <= 1000) {
            fsrForce = fsrConductance / 80;
        //      Serial.print("Force in Newtons: ");
        //      Serial.println(fsrForce);      
          } else {
            fsrForce = fsrConductance - 1000;
            fsrForce /= 30;
        //      Serial.print("Force in Newtons: ");
        //      Serial.println(fsrForce);            
          }
          
          int time_on = 500 + fsrForce * 10;
        //    Serial.println(time_on);

            if(control_input==0)
            {
              desired_angle = fsrForce*3;
            }
            
            step_size = int (stepsPerRevolution * desired_angle / 360.0);  
//            Serial.println(desired_angle);
//            Serial.println(step_size);
            myStepper.step(step_size);

        
            encoder_count=0;
            target_pos=0;
            integrator=0;
            last_encoder_count=0;
            speed_integrator=0;
            speed_target=0;
            break;
      }      
        /**************************/
    }

    static int loop_tele=0;
    if(loop_tele>=10)
    {
      loop_tele=0;

      send_telemetry();
    }
    else
    {
      loop_tele++;
    }

    
}


/***************************************End of Function section****************************/



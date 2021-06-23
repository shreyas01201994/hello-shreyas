#define EN        10       //atmega32// stepper motor enable, low level effective
#define X_DIR     14       //atmega32//X axis, stepper motor direction control 
#define Y_DIR     23       //atmega32//atmega32//y axis, stepper motor direction control
#define Z_DIR     2        //atmega32//z axis, stepper motor direction control                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
#define X_STP     15       //atmega32// x axis, stepper motor control
#define Y_STP     22       //atmega32//y axis, stepper motor control
#define Z_STP     3        //atmega32//z axis, stepper motor control
#define X_H_PIN   21    
#define X_F_PIN   20
#define Y_H_PIN   19
#define Y_F_PIN   18
#define Z_U_PIN   1  //upper side limit
#define Z_D_PIN   4 // down side limit
#define LED_PIN1  12
#define LED_PIN2  13
#define test_s     24
#define RED_LED    29
#define GREEN_LED  30
#define BLUE_LED   31

char limit_error,limit=0;
char pwm_val=0;
unsigned char LED_BYTE; 
unsigned char LED_BYTE2=90; 
unsigned char X_Y_SPEED;
unsigned char Z_SPEED;
const int stepsPerRevolution = 3200;
unsigned long Z_MAX_STEPS=48000; 


unsigned long z_count_position=0;
unsigned char z_up_pos_flag=0;
unsigned char z_down_pos_flag=0;
unsigned char x_h_position_flag=0;
unsigned char x_f_position_flag=0;
unsigned long x_count_position=0;
unsigned char y_h_position_flag=0;
unsigned char y_f_position_flag=0;
unsigned long y_count_position=0;
unsigned int MOTOR_DELAY=50;
unsigned int MOTOR_DELAY_XY=1000;


void GREEN_LIGHT_LED();
void RED_LIGHT_LED();
void BLUE_LIGHT_LED();

//The possible PWM frequencies of Pin 11 (higher than 488 Hz) are:

//31373 / 32 = 980.4 Hz
//31373 / 8 = 3921.6 Hz
//31373 / 1 = 31373 Hz

//Where 32, 8, 1 are the prescaler.
//Which equates to a setting value of 0x03, 0x02 & 0x01 respectively.

//Add this line of code:

//TCCR2B = TCCR2B & 0b11111000 | setting;

//Where setting is the value of the setting for the respective prescaler.

//============================================  
//|| Frequency [Hz] || Prescaler || Setting ||  
//============================================  
//|| 31373.55       || 1         || 0x01    ||  
//|| 3921.57        || 8         || 0x02    ||  
//|| 980.39         || 32        || 0x03    ||  
//|| 490.20         || 64        || 0x04    ||  
//|| 245.10         || 128       || 0x05    ||  
//|| 122.55         || 256       || 0x06    ||  
//|| 30.64          || 1024      || 0x07    ||  
//============================================  


/*
// Function: step   -control the direction and number of steps of the stepper motor
// Parameter: dir  -direction control, dirPin corresponds to DIR pin, stepperPin correspomds to 
Command Codes
DEC
  The command codes for the instructions of the Controller are given below. The software can send 100 and wait for an acknowledgement of 200 to verify that the controller is connected to the computer and powered on.  

Initialization Parameters for Serial Communication
Baud Rate: 19200 bps
Byte Size: 8
Parity: None
Stop Bits: 1
  
       X MOVE

    1. Send 101 to controller and wait for the acknowledgement 10.
    2. Send XByte2; ack XByte2.  //MSB
    3. Send XByte1; ack XByte1.  //CSB
    4. Send XByte0; ack XByte0.  //LSB
    5. Send direction code for X (1 for +ve & 0 for -ve); 
    6. Wait for any of the following codes 
  84 or 85 – Indicates X Home and FAR limit.    
  170 – Indicates that the movement is completed.


Notes

    1. XByte2, XByte1 & XByte0 are the millimeter to move for X axis broken into three bytes.  For example, if X has to be moved 0.5 mm, multiply with 1000 then convert to MSB, CSB and LSB.
Example: 0.5 *1000 = 500;
XByte2 = 0 (500/65536) 
XByte1 = 1 ((500% 65536) / 256));
XByte0 = 244 ((500% 65536) % 256));

           Reverse calculation 
           Move =500= (0*65536 + 1 *256 + 244)/1000;


       Y MOVE

1.Send 111 to controller and wait for the acknowledgement 10.
2.Send YByte2; ack YByte2.  //MSB
3.Send YByte1; ack YByte1.  //CSB
4.Send YByte0; ack YByte0.  //LSB
5. direction code for Y (1 for +ve & 0 for -ve); 
6.Wait for any of the following codes 
  86 or 87 – Indicates Y Home and FAR limit.    
  170 – Indicates that the movement is completed.

Notes

    2. YByte2, YByte1 & YByte0 are the millimeter to move for Y axis broken into three bytes.  For example, if Y has to be moved 0.5 mm, multiply with 1000 then convert to MSB, CSB and LSB.
Example: 0.5 *1000 = 500;
YByte2 = 0 (500/65536) t
YByte1 = 1 ((500% 65536) / 256));
YByte0 = 244 ((500% 65536) % 256));
          
           Reverse Calculation  
           Move =500= (0*65536 + 1 *256 + 244)/1000;

       Z MOVE

1.Send 121 to controller and wait for the acknowledgement 10.
2.Send ZByte2; ack ZByte2.  //MSB
3.Send ZByte1; ack ZByte1.  //CSB
4.Send ZByte0; ack ZByte0.  //LSB
      5. direction code for Z (1 for up & 0 for down); 
6.Wait for any of the following codes 
  88 or 89 – Indicates Z Home and FAR limit.    
  170 – Indicates that the movement is completed.


Notes

    1. ZByte2, ZByte1 & ZByte0 are the microns to move for Z axis broken into three bytes.  For example, if X has to be moved 0.5 microns, multiply with 100 then convert to MSB, CSB and LSB.
Example: 0.5 *100 = 50;
ZByte2 = 0 (50/65536) 
ZByte1 = 0 ((50% 65536) / 256));
ZByte0 = 50 ((50% 65536) % 256));
          
           Reverse Calculation  
           Move =50= (0*65536 + 0 *256 + 50)/100;

       LED 1

1.Send 82 to controller and wait for the acknowledgement 10.
2.Send Byte0; ack 10.  // 0 To 255 0 is low intensity and 255 high intensity 

       LED 2

1.Send 83 to controller and wait for the acknowledgement 10.
2.Send Byte0; ack 10.  // 0 To 255 0 is low intensity and 255 high intensity 

       SPEED X and Y
1.Send 81 to controller and wait for the acknowledgement 10.
2.Send Byte0; ack 10.  // 0 To 255 0 is low intensity and 255 high intensity 

       SPEED Z

1.Send 92 to controller and wait for the acknowledgement 10.
2.Send Byte0; ack 10.  // 0 To 255 0 is low intensity and 255 high intensity 

      AXIS MOVEMENT STOP

1.Send 131 to controller and wait for the acknowledgement 10.

step pin, steps is the number of steps.
// no return value
*/
int incomingByte = 0; // for incoming serial data

unsigned char step(boolean dir, byte dirPin, byte stepperPin, unsigned long steps)
{
  unsigned int delay_c=50;
  if(stepperPin== Z_STP) delay_c=MOTOR_DELAY;
  else delay_c=MOTOR_DELAY_XY;
  unsigned char limit_error=0;
  unsigned char sur=0;
  digitalWrite(dirPin, dir);
  delay(10);
  for (unsigned long i = 0; i < steps; i++) {
  if(!digitalRead(limit))
    {

      if(stepperPin== Z_STP)  digitalWrite(stepperPin, HIGH);
   else digitalWrite(stepperPin, LOW);  // CHANGE
    delayMicroseconds(delay_c);  
     if(stepperPin== Z_STP)  digitalWrite(stepperPin, LOW);
   else digitalWrite(stepperPin, HIGH); // CHANGE
    delayMicroseconds(delay_c);  
    if((z_up_pos_flag==1)&&(z_count_position > 0))
    {
      z_count_position--;   
    }
    else if(z_down_pos_flag==1)
    {
      z_count_position++;
    }
    else if((x_h_position_flag==1) && (x_count_position > 0))
    {
      x_count_position--;
    }
    else if(x_f_position_flag==1)
    {
      x_count_position++;
    }
    else if((y_h_position_flag==1) && (y_count_position > 0))
    {
      y_count_position--;
    }
    else if(y_f_position_flag==1)
    {
      y_count_position++;
    }
           
    limit_error=0;     
    }
    else 
   {
      if( limit_error<10)
      { if(i>0)i--;
        delayMicroseconds(10);  
         if(digitalRead(limit))
       {
       limit_error++;
      // limit_error=0;
       }
      }
      if(limit_error>9)
      {
       limit_error=1;
       if(limit==Z_U_PIN) z_count_position=0;
       else if(limit==X_H_PIN) x_count_position=0;
       else if(limit==Y_H_PIN) y_count_position=0;
       
       break;
      
      }
     
    }
  }
  
  return limit_error;
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// step_with_count////////////////////////////////////////////////////////////////////
void step_with_count(boolean dir, byte dirPin, byte stepperPin,unsigned long steps)
{
  unsigned char limit_error=0;
  digitalWrite(dirPin, dir);
  delay(10);
  for (unsigned long i = 0; i < steps; i++) {
  
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(50);  
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(50);  
  }
  return limit_error;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void step_test(boolean dir, byte dirPin, byte stepperPin, unsigned long distance)
{
  //unsigned long steps=distance*32;   
  unsigned long steps=192000;                                                       
  digitalWrite(dirPin, dir);
  delay(10);
  for (unsigned long  i = 0; i < steps; i++) {
 // while(1){
 // if(!digitalRead(limit))
   {
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(200);  
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(200); 
    //steps++; 
    }
    //else 
    {
     // limit_error=1;
     // break;
    }
  }
 unsigned char test_digit6=steps/1000000;
  unsigned char temp=steps%1000000;
  unsigned char test_digit5=steps/100000;
    temp=temp%100000;
  unsigned char test_digit4=temp/10000;
       temp= temp%10000;
  unsigned char test_digit3=temp/1000;
       temp= temp%1000; 
       
  unsigned char test_digit2=temp/100;
       temp= temp%100;
  unsigned char test_digit1=temp/10;         
   unsigned char test_digit0=temp%10;  

    Serial.print("\nstep covered:");
    Serial.println(test_digit6, DEC);
    Serial.println(test_digit5, DEC);
    Serial.println(test_digit4, DEC);
    Serial.println(test_digit3, DEC);
    Serial.println(test_digit2, DEC);
    Serial.println(test_digit1, DEC);
    Serial.println(test_digit0, DEC); 
}



/*
Initialization Parameters for Serial Communication
Baud Rate: 19200 bps
Byte Size: 8
Parity: None
Stop Bits: 1
*/
void setup(){// set the IO pins for the stepper motors as output 
  pinMode(X_DIR, OUTPUT); pinMode(X_STP, OUTPUT);
  pinMode(Y_DIR, OUTPUT); pinMode(Y_STP, OUTPUT);
  pinMode(Z_DIR, OUTPUT); pinMode(Z_STP, OUTPUT);


   pinMode(29, OUTPUT);
   pinMode(30, OUTPUT);
   pinMode(31, OUTPUT);
  //pinMode(EN, OUTPUT);
  
   pinMode(EN, OUTPUT);
    pinMode(X_H_PIN, INPUT);
    pinMode(X_F_PIN, INPUT);
    pinMode(Y_H_PIN, INPUT);
    pinMode(Y_F_PIN, INPUT);
    pinMode(Z_U_PIN, INPUT);
    pinMode(Z_D_PIN, INPUT);
    
    pinMode(LED_PIN1, OUTPUT);
    pinMode(LED_PIN2, OUTPUT);
   
  digitalWrite(EN, LOW);   //change
  Serial.begin(9600);           // set up Serial library at 9600 bps 
  delay(500);
  Serial.println("Autoscope initiated!"); 
 //analogWrite(11,70);
   digitalWrite(RED_LED, HIGH);//red29 
   digitalWrite(GREEN_LED, HIGH);//Green30
   digitalWrite(BLUE_LED, HIGH);//Blue31

   GREEN_LIGHT_LED();
   pinMode(test_s, OUTPUT);
    digitalWrite(test_s, LOW);   
LED_BYTE = 255;
LED_BYTE2 = 0;
 // pwm_val=64;
 // LED_BYTE = 150;
 analogWrite(LED_PIN1,LED_BYTE);
 analogWrite(LED_PIN2,LED_BYTE2);
}

void anticlockwise_test(unsigned long dist)
{
  limit=9;
  step_test(true, X_DIR, X_STP, dist); // X axis motor rotates CW for 1 circle, as in 200 steps
  if(limit_error==1)
  Serial.print("1: limit reached "); 
  else  Serial.print("1: anticlock "); 
   limit_error=0;
}

void clockwise_test(unsigned long dist)
{
  limit=10;
  step_test(false, X_DIR, X_STP, dist); // X axis motor rotates CW for 1 circle, as in 200 steps
  if(limit_error==1)
  Serial.print("2: limit reached "); 
  else  Serial.print("2: clock "); 
  limit_error=0;
}

void anticlockwise_test_y(unsigned long dist)
{
  limit=12;
  step_test(true, Y_DIR, Y_STP, dist); // y axis motor rotates CW for 1 circle, as in 200 steps
  if(limit_error==1)
  Serial.print("1: limit reached "); 
  else  Serial.print("1: anticlock "); 
   limit_error=0;
}

void clockwise_test_y(unsigned long dist)
{
  limit=11;
  step_test(false, Y_DIR, Y_STP, dist); // y axis motor rotates CW for 1 circle, as in 200 steps
  if(limit_error==1)
  Serial.print("2: limit reached "); 
  else  Serial.print("2: clock "); 
  limit_error=0;
}




void anticlockwise(){
  step(true, X_DIR, X_STP, 3200); // X axis motor rotates CW for 1 circle, as in 200 steps
  step(true, Y_DIR, Y_STP, 3200); // y axis motor rotates CW for 1 circle, as in 200 steps
  step(true, Z_DIR, Z_STP, 3200); // z axis motor rotates CW for 1 circle, as in 200 steps
  }
void clockwise(){
  step(false, X_DIR, X_STP, 3200); // x axis motor rotates CCW for 1 circle, as in 200 steps
  step(false, Y_DIR, Y_STP, 3200); // y axis motor rotates CCW for 1 circle, as in 200 steps
  step(false, Z_DIR, Z_STP, 3200); // z axis motor rotates CCW for 1 circle, as in 200 steps
  }  
  
void loop(){

    // say what you got:
   // Serial.print("Received: ");
   // Serial.println(incomingByte, DEC);
  
   


 
 
if(Serial.available())
{
  incomingByte=Serial.read();
  
  switch(incomingByte)
  {
    case  101:BLUE_LIGHT_LED();
              x_axis_protocol();
              GREEN_LIGHT_LED();
             break;  

    case  111:BLUE_LIGHT_LED();
             y_axis_protocol(); 
             GREEN_LIGHT_LED();
             break;           

    case 121:BLUE_LIGHT_LED();
             digitalWrite(EN, LOW);   //change
             z_axis_protocol();
             digitalWrite(EN, LOW);
             GREEN_LIGHT_LED();
             break;

    case  82:BLUE_LIGHT_LED();
             LED_BYTE=led_protocol();
             analogWrite(LED_PIN1,LED_BYTE);
             GREEN_LIGHT_LED();
             break;  

    case 91:BLUE_LIGHT_LED();
            LED_BYTE2=led_protocol();
            analogWrite(LED_PIN2,LED_BYTE2);
            GREEN_LIGHT_LED();
            break; 

    case 93: BLUE_LIGHT_LED();
             Serial.write(10);    
             LED_BYTE2=0;
             analogWrite(LED_PIN2,0);
             GREEN_LIGHT_LED();
             break; 

   // case  81:X_Y_SPEED=x_y_speed_protocol();
    //         break; 

   // case  91:Z_SPEED=z_speed_protocol();
   //          break;    

    case  70:BLUE_LIGHT_LED();
             x_home_protocol();
             GREEN_LIGHT_LED();
             break;  
             
    case  67:BLUE_LIGHT_LED();
             x_far_protocol();
             GREEN_LIGHT_LED();
             break; 
             
    case  71:BLUE_LIGHT_LED();
             y_home_protocol();
             GREEN_LIGHT_LED();
             break;
                   
    case  68:BLUE_LIGHT_LED();
             y_far_protocol();
             GREEN_LIGHT_LED();
             break;

    case  72:BLUE_LIGHT_LED();
             z_up_count();
             GREEN_LIGHT_LED();
             break;
             
    case  69:BLUE_LIGHT_LED();
             z_down_count();
             GREEN_LIGHT_LED();
             break;

    case  48: BLUE_LIGHT_LED();
              x_axis_protocol_2();  //
              GREEN_LIGHT_LED();
              break;

    case  49: BLUE_LIGHT_LED();
              y_axis_protocol_2();  // 1
              GREEN_LIGHT_LED();
              break;

    case  50:  BLUE_LIGHT_LED();
               digitalWrite(EN, LOW);   //change
               z_axis_protocol_2();  //2
               digitalWrite(EN, LOW);
               GREEN_LIGHT_LED();
               break;

   case  51:  BLUE_LIGHT_LED();
              dispaly_z_position();  //3
              GREEN_LIGHT_LED();
              break; 

   case  52:  BLUE_LIGHT_LED();
               send_z_position();     //4
               GREEN_LIGHT_LED();
               break; 

   case  53:BLUE_LIGHT_LED();
           dispaly_x_position();  //5
           GREEN_LIGHT_LED();
           break; 

    case  54:BLUE_LIGHT_LED();
             send_x_position();     //6
             GREEN_LIGHT_LED();
             break;          

    case  55:BLUE_LIGHT_LED();
             dispaly_y_position();  //7
             GREEN_LIGHT_LED();
             break;
              
    case  56: BLUE_LIGHT_LED();
              send_y_position();     //8  
              GREEN_LIGHT_LED();
              break;                             
             

    case  81:BLUE_LIGHT_LED();
             MOTOR_DELAY_XY=motor_speed_protocol();
             GREEN_LIGHT_LED();
             break; 

    case  92:BLUE_LIGHT_LED();
             MOTOR_DELAY=motor_speed_protocol();
             GREEN_LIGHT_LED();
             break; 
                     

   case  '?':Serial.print("working shreyas");  //63
             break; 
             
   default : RED_LIGHT_LED();
             delay(1000);
             GREEN_LIGHT_LED();
             
  }
}

    
   /* if(incomingByte == '1'){
       if(!digitalRead(10))
       {
        Serial.print("1: move clockwise "); 
        anticlockwise();
       }
       else Serial.print("1: Limit Reached "); 
      delay(10);
    }
    else if(incomingByte == '2'){
      if(!digitalRead(9))
       {
        Serial.print("2: move clockwise "); 
        clockwise();
       }
       else Serial.print("2: Limit Reached "); 
      
      delay(10);
    }*/
  }
  


void x_axis_protocol()
{
  unsigned char  XMSB=0;
  unsigned char  XCSB=0;
  unsigned char  XLSB=0;
  unsigned char  XDIR=0;
  
    Serial.write(10);                      // ack send to PC
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                  XMSB = Serial.read();
                Serial.write(XMSB);                  // ack send to PC
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                 XCSB = Serial.read();
                Serial.write(XCSB);                  // ack send to PC
                
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                XLSB = Serial.read();
                Serial.write(XLSB);                  // ack send to PC
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
           {
                 XDIR = Serial.read();  
                 if(XDIR=='1')limit=X_F_PIN;
                 else limit=X_H_PIN;
                 
             }

           unsigned char error=  x_axis_run( XMSB, XCSB, XLSB, XDIR);

           x_h_position_flag=0;
           x_f_position_flag=0;
          
          if(error && limit==X_F_PIN )Serial.write(84); 
           else if  (error && limit==X_H_PIN) Serial.write(85); 
           else if (!error) Serial.write(170); 
}

 unsigned char x_axis_run(unsigned char xmsb,unsigned char xcsb,unsigned char xlsb,unsigned char dir)
{
 
  if(dir) 
  {limit=X_F_PIN;
   x_f_position_flag=1;
  }
  else 
  {limit=X_H_PIN;
   x_h_position_flag=1;
  }
  unsigned long xlsb_c=(unsigned long)xlsb;
  unsigned long xcsb_c=(unsigned long)xcsb;
  unsigned long xmsb_c=(unsigned long)xmsb;
 unsigned long x_distance= (x_distance | xlsb_c) | (x_distance | (xcsb_c<<8)) | ( x_distance | (xmsb_c<<16));
 unsigned long cal_steps=(x_distance*32)/10;
  digitalWrite(test_s, HIGH);  
  unsigned char error=step(dir, X_DIR,  X_STP, cal_steps);
   digitalWrite(test_s, LOW);  
  return error;
}

/////////////////////////  y protocol /////////////////////////////////////////
void y_axis_protocol()
{
  unsigned char  YMSB=0;
  unsigned char  YCSB=0;
  unsigned char  YLSB=0;
  unsigned char  YDIR=0;
  
    Serial.write(10);                      // ack send to PC
     
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                  YMSB = Serial.read();
                Serial.write(YMSB);                  // ack send to PC /0
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                 YCSB = Serial.read();
                Serial.write(YCSB);                  // ack send to PC               
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                YLSB = Serial.read();
                Serial.write(YLSB);                  // ack send to PC    
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                 YDIR = Serial.read();  
                  if(YDIR)limit=Y_F_PIN;
                 else limit=Y_H_PIN;
             }

          unsigned char error=  y_axis_run( YMSB, YCSB, YLSB, YDIR);

          y_f_position_flag=0;
          y_h_position_flag=0;
          
           if(error && limit==Y_F_PIN )Serial.write(86); 
           else if  (error && limit==Y_H_PIN) Serial.write(87); 
           else if (!error) Serial.write(170);    
    
}

 unsigned char y_axis_run(unsigned char ymsb,unsigned char ycsb,unsigned char ylsb,unsigned char dir)
{
  
  if(dir) 
  {limit=Y_F_PIN;
  y_f_position_flag=1;
  }
  else 
  {limit=Y_H_PIN;
  y_h_position_flag=1;
  }
  unsigned long ylsb_c=(unsigned long)ylsb;
  unsigned long ycsb_c=(unsigned long)ycsb;
  unsigned long ymsb_c=(unsigned long)ymsb;
 unsigned long y_distance= (y_distance | ylsb_c) | (y_distance | (ycsb_c<<8)) | ( y_distance | (ymsb_c<<16));
 unsigned long cal_steps=(y_distance*32)/10;
  digitalWrite(test_s, HIGH);  
  unsigned char error1=step(dir, Y_DIR,  Y_STP, cal_steps);
   digitalWrite(test_s, LOW);  
  return error1;
}
///////////////////////////////////////////////////////////////////////////////////////////
///


////////////////////////////   z axis protocol /////////////////////////////////////////
void z_axis_protocol()
{
  unsigned char  ZMSB=0;
  unsigned char  ZCSB=0;
  unsigned char  ZLSB=0;
  unsigned char  ZDIR=0;
  
    Serial.write(48);                      // ack send to PC
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                  ZMSB = Serial.read();
                Serial.write(ZMSB);                  // ack send to PC
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                 ZCSB = Serial.read();
                Serial.write(ZCSB);                  // ack send to PC
                
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                ZLSB = Serial.read();
                Serial.write(ZLSB);                  // ack send to PC
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                 ZDIR = Serial.read();  
                 if(ZDIR)limit=Z_U_PIN;
                 else limit=Z_D_PIN;
             }

           unsigned char error=  z_axis_run( ZMSB, ZCSB, ZLSB, ZDIR);

           z_up_pos_flag=0;
           z_down_pos_flag=0;
           if(error && limit==Z_U_PIN )Serial.write(88); 
           else if  (error && limit==Z_D_PIN) Serial.write(89); 
           else if (!error) Serial.write(170); 
    
}

 unsigned char z_axis_run(unsigned char zmsb,unsigned char zcsb,unsigned char zlsb,unsigned char dir)
{
  
  if(dir)  
  {
    limit=Z_U_PIN;
    z_up_pos_flag=1;
    
  }
  else 
  {
    limit=Z_D_PIN;
    z_down_pos_flag=1;
    
  }
  unsigned long zlsb_c=(unsigned long)zlsb;
  unsigned long zcsb_c=(unsigned long)zcsb;
  unsigned long zmsb_c=(unsigned long)zmsb;
 unsigned long z_distance= (z_distance | zlsb_c) | (z_distance | (zcsb_c<<8)) | ( z_distance | (zmsb_c<<16));
 unsigned long cal_steps=(z_distance*32)/10;
  unsigned char error=step(dir, Z_DIR,  Z_STP, cal_steps);
  return error;
}




/////////////////////////////////////////// LED BRIGHTNESS ////////////////////////////////////////////


unsigned char led_protocol()
{
  unsigned char led_data;
 Serial.write(10);   
 while(!Serial.available()){}
 if (Serial.available() > 0) 
   {
    led_data= Serial.read();
    Serial.write(10);                  // ack send to PC
   }
  return led_data;
}


////////////////////// X Y SPEED  ///////////////////////////////////////////////

unsigned char x_y_speed_protocol()
{
 unsigned char x_y_speed;
  Serial.write(10);   
 while(!Serial.available()){}
 if (Serial.available() > 0) 
   {
     x_y_speed= Serial.read();
    Serial.write(10);                  // ack send to PC
   }
   return x_y_speed;
}


///////////////////// z axis speed //////////////////////////////////
unsigned char z_speed_protocol()
{
  unsigned char z_speed;
  Serial.write(10);   
 while(!Serial.available()){}
 if (Serial.available() > 0) 
   {
     z_speed= Serial.read();
    Serial.write(10);                  // ack send to PC
   }
   return z_speed;
}
////////////////////////////////////////////////////////////////////////

//////////////////////////////  x home protocol/////////////////////////
void x_home_protocol()
{
  x_home();
   Serial.write(10);  
}
/////////////////////////////////////////////////////////////////////////
void x_home()
{
  limit=X_H_PIN;

  digitalWrite(X_DIR, false);

  while(1) {
    if(!digitalRead(limit))
    {
    digitalWrite(X_STP, HIGH);
    delayMicroseconds(200);  
    digitalWrite(X_STP, LOW);
    delayMicroseconds(200);  
    limit_error=0;
    }
    else 
    {
      limit_error=1;
      break;
    } 
}
}
/////////////////////////////////////////////////////////////////////////////////

//////////////////////////////  x far protocol/////////////////////////
void x_far_protocol()
{
  x_far();
   Serial.write(10);  
}
/////////////////////////////////////////////////////////////////////////
void x_far()
{
  limit=X_F_PIN;

  digitalWrite(X_DIR, true);

  while(1) {
    if(!digitalRead(limit))
    {
    digitalWrite(X_STP, HIGH);
    delayMicroseconds(200);  
    digitalWrite(X_STP, LOW);
    delayMicroseconds(200);  
    limit_error=0;
    }
    else 
    {
      limit_error=1;
      break;
    } 
}
}
/////////////////////////////////////////////////////////////////////////////////


//////////////////////////////  y home protocol/////////////////////////
void y_home_protocol()
{
  y_home();
   Serial.write(10);  
}
/////////////////////////////////////////////////////////////////////////
void y_home()
{
  limit=Y_H_PIN;

  digitalWrite(Y_DIR, false);

  while(1) {
    if(!digitalRead(limit))
    {
    digitalWrite(Y_STP, HIGH);
    delayMicroseconds(200);  
    digitalWrite(Y_STP, LOW);
    delayMicroseconds(200);  
    limit_error=0;
    }
    else 
    {
      limit_error=1;
      break;
    } 
}
}
/////////////////////////////////////////////////////////////////////////////////

//////////////////////////////  Y far protocol/////////////////////////
void y_far_protocol()
{
  y_far();
   Serial.write(10);  
}
/////////////////////////////////////////////////////////////////////////
void y_far()
{
  limit=Y_F_PIN;

  digitalWrite(Y_DIR, true);

  while(1) {
    if(!digitalRead(limit))
    {
    digitalWrite(Y_STP, HIGH);
    delayMicroseconds(200);  
    digitalWrite(Y_STP, LOW);
    delayMicroseconds(200);  
    limit_error=0;
    }
    else 
    {
      limit_error=1;
      break;
    } 
}
}
///////////////////////////////////////////////////////////////////////////////


void z_up_count()
{
  step_with_count(true, Z_DIR, Z_STP,Z_MAX_STEPS);
}

///////////////////////////////////////////////////////////////////////////////
void z_down_count()
{
  step_with_count(false, Z_DIR, Z_STP,Z_MAX_STEPS);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void z_up_limit()
{
  limit=Z_U_PIN;
unsigned char error=step(true,Z_DIR,Z_STP,Z_MAX_STEPS);
}
///////////////////////////////////////////////////////////////////////////////

void z_down_limit()
{
  limit=Z_D_PIN;
unsigned char error=step(false,Z_DIR,Z_STP,Z_MAX_STEPS);
}
///////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void x_axis_protocol_2()
{
  unsigned char  XMSB=0;
  unsigned char  XCSB=0;
  unsigned char  XLSB=0;
  unsigned char  XDIR=0;
  
    Serial.write(10);                      // ack send to PC
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                  XMSB = Serial.read();
                Serial.write(XMSB);                  // ack send to PC
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                 XCSB = Serial.read();
                Serial.write(XCSB);                  // ack send to PC
                
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                XLSB = Serial.read();
                Serial.write(XLSB);                  // ack send to PC
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                 XDIR = Serial.read();  
                 if(XDIR=='1')limit=X_F_PIN;
                 else limit=X_H_PIN;
                 
             }

           unsigned char error=  x_axis_run_2( XMSB, XCSB, XLSB, XDIR);

           x_h_position_flag=0;
           x_f_position_flag=0;

           /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          
          if(error && limit==X_F_PIN )Serial.write(84); 
           else if  (error && limit==X_H_PIN) Serial.write(85); 
           else if (!error) Serial.write(126); 
}

 unsigned char x_axis_run_2(unsigned char xmsb,unsigned char xcsb,unsigned char xlsb,unsigned char dir)
{
 
  if(dir=='1') 
  {limit=X_F_PIN;
  dir=1;
  x_f_position_flag=1;
  }
  else 
  {limit=X_H_PIN;
  dir=0;
  x_h_position_flag=1;
  }
 
 unsigned long xlsb_c =(unsigned long)(xlsb-48);
 unsigned long xcsb_c =(unsigned long)(xcsb-48);
 unsigned long xmsb_c =(unsigned long)(xmsb-48);
   
  unsigned long  cal_steps= (xlsb_c + (xcsb_c*10) + (xmsb_c*100)) *3200;
    digitalWrite(test_s, HIGH);  
  unsigned char error=step(dir, X_DIR,  X_STP, cal_steps);
   digitalWrite(test_s, LOW);  
  return error;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void y_axis_protocol_2()
{
  unsigned char  YMSB=0;
  unsigned char  YCSB=0;
  unsigned char  YLSB=0;
  unsigned char  YDIR=0;
  
    Serial.write(10);                      // ack send to PC
     
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                  YMSB = Serial.read();
                Serial.write(YMSB);                  // ack send to PC 
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                 YCSB = Serial.read();
                Serial.write(YCSB);                  // ack send to PC               
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                YLSB = Serial.read();
                Serial.write(YLSB);                  // ack send to PC    
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                 YDIR = Serial.read();  
                  if(YDIR)limit=Y_F_PIN;
                 else limit=Y_H_PIN;
             }

          unsigned char error=  y_axis_run_2( YMSB, YCSB, YLSB, YDIR);

          y_f_position_flag=0;
          y_h_position_flag=0;
          
           if(error && limit==Y_F_PIN )Serial.write(86); 
           else if  (error && limit==Y_H_PIN) Serial.write(87); 
           else if (!error) Serial.write(126);    
    
}

 unsigned char y_axis_run_2(unsigned char ymsb,unsigned char ycsb,unsigned char ylsb,unsigned char dir)
{
  
  if(dir=='1') 
  {limit=Y_F_PIN;
  dir=1;
  y_f_position_flag=1;
  }
  else 
  {limit=Y_H_PIN;
   dir=0;
   y_h_position_flag=1;
  }
  unsigned long ylsb_c =(unsigned long)(ylsb-48);
 unsigned long ycsb_c =(unsigned long)(ycsb-48);
 unsigned long ymsb_c =(unsigned long)(ymsb-48);
   
  unsigned long  cal_steps= (ylsb_c + (ycsb_c*10) + (ymsb_c*100)) *3200;
    digitalWrite(test_s, HIGH);  
  unsigned char error1=step(dir, Y_DIR,  Y_STP, cal_steps);
   digitalWrite(test_s, LOW);  
  return error1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void z_axis_protocol_2()
{
  unsigned char  ZMSB=0;
  unsigned char  ZCSB=0;
  unsigned char  ZLSB=0;
  unsigned char  ZDIR=0;
  
    Serial.write(10);                      // ack send to PC
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                  ZMSB = Serial.read();
                Serial.write(ZMSB);                  // ack send to PC
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                 ZCSB = Serial.read();
                Serial.write(ZCSB);                  // ack send to PC
                
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                ZLSB = Serial.read();
                Serial.write(ZLSB);                  // ack send to PC
             }
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                 ZDIR = Serial.read();  
                 if(ZDIR)limit=Z_U_PIN;
                 else limit=Z_D_PIN;
             }

           unsigned char error=  z_axis_run_2( ZMSB, ZCSB, ZLSB, ZDIR);
           z_up_pos_flag=0;
           z_down_pos_flag=0;
           if(error && limit==Z_U_PIN )Serial.write(88); 
           else if  (error && limit==Z_D_PIN) Serial.write(89); 
           else if (!error) Serial.write(126); 
    
}

 unsigned char z_axis_run_2(unsigned char zmsb,unsigned char zcsb,unsigned char zlsb,unsigned char dir)
{
  
  if(dir=='1')  
  {
    limit=Z_U_PIN;
    dir=1;
    z_up_pos_flag=1;
  }
  else 
  {
    limit=Z_D_PIN;
    dir=0;
    z_down_pos_flag=1;
  }
  unsigned long zlsb_c =(unsigned long)(zlsb-48);
  unsigned long zcsb_c =(unsigned long)(zcsb-48);
  unsigned long zmsb_c =(unsigned long)(zmsb-48);
   
  unsigned long  cal_steps= (zlsb_c + (zcsb_c*10) + (zmsb_c*100)) *3200;
  
  unsigned char error=step(dir, Z_DIR,  Z_STP, cal_steps);
  return error;
}



unsigned int motor_speed_protocol()
{
  unsigned char motor_speed=0;
  
             Serial.write(10);                      // ack send to PC
             while(!Serial.available()){}
             if (Serial.available() > 0) 
             {
                  motor_speed = Serial.read();
                Serial.write(motor_speed);                  
             }

              

       //  unsigned int motor_delay=(unsigned int)((motor_speed*480)/255);
         ////             motor_delay=(480-motor_delay)+20;
           //        if(motor_speed==255) motor_delay=20;
        unsigned int   motor_delay = (unsigned int)((unsigned int)((255-motor_speed)*5)+30);
                       
                       
                       if(motor_speed==0)motor_delay=30;

                       
                       
                     



    
 /* unsigned char test_digit4=(unsigned char)(motor_delay/10000);
  
   unsigned long    temp= motor_delay%10000;
  unsigned char test_digit3=(unsigned char)(temp/1000);
       temp= temp%1000; 
       
  unsigned char test_digit2=(unsigned char)(temp/100);
       temp= temp%100;
  unsigned char test_digit1=(unsigned char)(temp/10);         
   unsigned char test_digit0=(unsigned char)(temp%10);  

    Serial.println("\delay moto:");
    
    Serial.print(test_digit4, DEC);
    Serial.print(test_digit3, DEC);
    Serial.print(test_digit2, DEC);
    Serial.print(test_digit1, DEC);
    Serial.print(test_digit0, DEC); */
                   return motor_delay; 

                           
}


////////////////////////////////////////  z axis position  ///////////////////////////////////////////////////////
void dispaly_z_position()
{    
  unsigned char test_digit6=(unsigned char)(z_count_position/1000000);
  unsigned long temp= (z_count_position%1000000);
  unsigned char test_digit5= (unsigned char) (temp/100000);
    temp=temp%100000;
  unsigned char test_digit4=(unsigned char)(temp/10000);
       temp= temp%10000;
  unsigned char test_digit3=(unsigned char)(temp/1000);
       temp= temp%1000; 
       
  unsigned char test_digit2=(unsigned char)(temp/100);
       temp= temp%100;
  unsigned char test_digit1=(unsigned char)(temp/10);         
   unsigned char test_digit0=(unsigned char)(temp%10);  

    Serial.println("\z position:");
    Serial.print(test_digit6, DEC);
    Serial.print(test_digit5, DEC);
    Serial.print(test_digit4, DEC);
    Serial.print(test_digit3, DEC);
    Serial.print(test_digit2, DEC);
    Serial.print(test_digit1, DEC);
    Serial.print(test_digit0, DEC); 
}

void send_z_position()
{
  unsigned char  ZMSB=0;
  unsigned char  ZCSB=0;
  unsigned char  ZLSB=0;

ZLSB= (unsigned char)(z_count_position & 0x000000FF);
ZCSB= (unsigned char)( (z_count_position>>8) & 0x000000FF);
ZMSB= (unsigned char)( (z_count_position>>16) & 0x000000FF);
  
  Serial.write(ZMSB); 
  Serial.write(ZCSB); 
  Serial.write(ZLSB);  
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////// x axis position /////////////////////////////////////////
void dispaly_x_position()
{    
  unsigned char test_digit6=(unsigned char)(x_count_position/1000000);
  unsigned long temp= (x_count_position%1000000);
  unsigned char test_digit5= (unsigned char) (temp/100000);
    temp=temp%100000;
  unsigned char test_digit4=(unsigned char)(temp/10000);
       temp= temp%10000;
  unsigned char test_digit3=(unsigned char)(temp/1000);
       temp= temp%1000; 
       
  unsigned char test_digit2=(unsigned char)(temp/100);
       temp= temp%100;
  unsigned char test_digit1=(unsigned char)(temp/10);         
   unsigned char test_digit0=(unsigned char)(temp%10);  

    Serial.println("X position:");
    Serial.print(test_digit6, DEC);
    Serial.print(test_digit5, DEC);
    Serial.print(test_digit4, DEC);
    Serial.print(test_digit3, DEC);
    Serial.print(test_digit2, DEC);
    Serial.print(test_digit1, DEC);
    Serial.print(test_digit0, DEC); 
}

void send_x_position()
{
  unsigned char  XMSB=0;
  unsigned char  XCSB=0;
  unsigned char  XLSB=0;

XLSB= (unsigned char)(x_count_position & 0x000000FF);
XCSB= (unsigned char)( (x_count_position>>8) & 0x000000FF);
XMSB= (unsigned char)( (x_count_position>>16) & 0x000000FF);
  
  Serial.write(XMSB); 
  Serial.write(XCSB); 
  Serial.write(XLSB);   
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////// Y axis position /////////////////////////////////////////
void dispaly_y_position()
{    
  // max 60mm distance full y axis       3200 -> 1mm    hence     3200*60= 192000 counts for full y or x axis distance
  unsigned char test_digit6=(unsigned char)(y_count_position/1000000);
  unsigned long temp= (y_count_position%1000000);
  unsigned char test_digit5= (unsigned char) (temp/100000);
    temp=temp%100000;
  unsigned char test_digit4=(unsigned char)(temp/10000);
       temp= temp%10000;
  unsigned char test_digit3=(unsigned char)(temp/1000);
       temp= temp%1000; 
       
  unsigned char test_digit2=(unsigned char)(temp/100);
       temp= temp%100;
  unsigned char test_digit1=(unsigned char)(temp/10);         
   unsigned char test_digit0=(unsigned char)(temp%10);  

    Serial.println("Y position:");
    Serial.print(test_digit6, DEC);
    Serial.print(test_digit5, DEC);
    Serial.print(test_digit4, DEC);
    Serial.print(test_digit3, DEC);
    Serial.print(test_digit2, DEC);
   // Serial.print(   , DEC);
    Serial.print(test_digit0, DEC); 
    
}

void send_y_position()
{
  unsigned char  YMSB=0;
  unsigned char  YCSB=0;
  unsigned char  YLSB=0;

YLSB= (unsigned char)(y_count_position & 0x000000FF);
YCSB= (unsigned char)( (y_count_position>>8) & 0x000000FF);
YMSB= (unsigned char)( (y_count_position>>16) & 0x000000FF);
  
  Serial.write(YMSB); 
  Serial.write(YCSB); 
  Serial.write(YLSB);   
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////

void GREEN_LIGHT_LED()
{
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED,HIGH);
}

void RED_LIGHT_LED()
{
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED,LOW);
}
void BLUE_LIGHT_LED()
{
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED,HIGH);
}

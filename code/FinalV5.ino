//========================================================Author:SLOMOGANGSTA(VSYK)===========================================================
//========================================================Date: 27th January 2019=============================================================
#include<Wire.h>

byte clockspeed_ok=0;
int gyro_x=0, gyro_y=0, gyro_z=0;
long acc_x=0, acc_y=0, acc_z=0, acc_total_vector=0;
int temperature=0;
long gyro_x_cal=0, gyro_y_cal=0, gyro_z_cal=0;
float angle_pitch=0, angle_roll=0;
boolean set_gyro_angles,auto_level;
float angle_roll_acc=0, angle_pitch_acc=0;
float angle_pitch_output=0, angle_roll_output=0;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4;
float roll_level_adjust=0, pitch_level_adjust=0;
float pid_error_temp=0;
float pid_i_mem_roll=0, pid_roll_setpoint=0, gyro_roll_input=0, pid_output_roll=0, pid_last_roll_d_error=0;
float pid_i_mem_pitch=0, pid_pitch_setpoint=0, gyro_pitch_input=0, pid_output_pitch=0, pid_last_pitch_d_error=0;
float pid_i_mem_yaw=0, pid_yaw_setpoint=0, gyro_yaw_input=0, pid_output_yaw=0, pid_last_yaw_d_error=0;
int receiver_input_channel_1=0, receiver_input_channel_2=0, receiver_input_channel_3=0, receiver_input_channel_4=0;
int loopCounter = 0;
unsigned long loop_start_time = 0;
float pid_p_gain_roll = 0;               
float pid_i_gain_roll = 0;               
float pid_d_gain_roll = 0;               
int   pid_max_roll = 400;                 
float pid_p_gain_pitch = 0;             
float pid_i_gain_pitch = 0;             
float pid_d_gain_pitch = 0;              
int pid_max_pitch = 400;  
float pid_p_gain_yaw = 0;                
float pid_i_gain_yaw = 0;               
float pid_d_gain_yaw = 0;
int pid_max_yaw = 400;                                   
int pid_max_yaw_i = 100;                   
int servoVal1 , servoVal2 , servoVal3 , servoVal4;
float pinten =0;              
               
void setup()
{
  Serial.begin(57600);
  Wire.begin();      
  
  PCICR |= (1 << PCIE0);                                                    
  PCMSK0 |= (1 << PCINT0);                                                
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);                                                
  PCMSK0 |= (1 << PCINT3);
  
  Serial.println("Interrupts Enabled Successfully"); 
  delay(100); 
    
  Serial.println(F("Checking I2C clock speed."));
  delay(100);
  TWBR = 12;                    
  
  #if F_CPU == 16000000L          
    clockspeed_ok = 1;            
  #endif                    

  if(TWBR == 12 && clockspeed_ok)
  {
    Serial.println(F("I2C clock speed is correctly set to 400kHz."));
  }
  else
  {
    Serial.println(F("I2C clock speed is not set to 400kHz. (ERROR 8)"));
    exit(0);
  }

  Serial.println("Setting DIGITAL PIN 4 , 5 , 6 ,7 as OUTPUTS");
   DDRD |= B11110000;
  
  Serial.println("Setting up registers of MPU6050");
  setup_mpu_6050_registers();

  Serial.println("Calculating Offset ");
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++)
  {   
    if(cal_int % 125 == 0)Serial.print(".");                            
    read_mpu_6050_data();                                              
    gyro_x_cal += gyro_x;                                             
    gyro_y_cal += gyro_y;                                            
    gyro_z_cal += gyro_z;                                             
    delay(3);                                                       
  }
  gyro_x_cal /= 2000;                                                  
  gyro_y_cal /= 2000;                                              
  gyro_z_cal /= 2000; 

  Serial.println("Calculated Offsets are");
  Serial.print("Pitch offset: ");
  Serial.println(gyro_x_cal);
  Serial.print("Roll offset: ");
  Serial.println(gyro_y_cal);
  Serial.print("Yaw offset: ");
  Serial.println(gyro_z_cal);
}

void loop()
{
  //Serial.print("Receiver Input 3: ");
  //Serial.println(receiver_input_channel_3);
  if(receiver_input_channel_4 < 1000)
  {
   pid_d_gain_roll = 0;
   pid_p_gain_roll = 0;
   pid_d_gain_pitch = 0;
   pid_p_gain_pitch = 0;
   pid_p_gain_yaw = 0;
   pid_d_gain_yaw = 0;
   //Serial.println("INTENSITY : 0"); 
  }
   
  else
  {
  pinten = mapf(receiver_input_channel_4 , 1000 , 2050 , 0.07 , 0.17 );
  pid_d_gain_roll = pinten;
  pid_p_gain_roll = pinten;
  pid_d_gain_pitch = pinten;
  pid_p_gain_pitch = pinten;
  pid_p_gain_yaw = pinten;
  pid_d_gain_yaw = pinten;
  //Serial.print("INTENSITY: ");
  //Serial.println(pinten);
  }
  
  read_mpu_6050_data();                                                

  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
  
  angle_pitch += gyro_x * 0.0000611;                                   
  angle_roll += gyro_y * 0.0000611;                                    
  
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               
  
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;      
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;     
  

  angle_pitch_acc -= 0.0;                                              
  angle_roll_acc -= 0.0;                                           

  if(set_gyro_angles)
  {                                    
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;     
  }
  else
  {                                                          
    angle_pitch = angle_pitch_acc;                                    
    angle_roll = angle_roll_acc;                                     
    set_gyro_angles = true;                                           
  }
  
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;

  /*Serial.print("Pitch : ");
  Serial.println(angle_pitch_output);
  Serial.print("Roll : ");
  Serial.println(angle_roll_output);*/

  pitch_level_adjust = angle_pitch_output * 15;                                    
  roll_level_adjust = angle_roll_output * 15;                                      

  float uptake = 0.2;
  float oneMinusUptake = 1 - uptake;
  
  gyro_pitch_input = (gyro_pitch_input * oneMinusUptake) + (gyro_x * uptake);
  gyro_roll_input  = (gyro_roll_input * oneMinusUptake)  + (gyro_y * uptake);
  gyro_yaw_input   = (gyro_yaw_input * oneMinusUptake)   + (gyro_z * uptake);


  pid_roll_setpoint = 0;
  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;
  pid_roll_setpoint -= roll_level_adjust;                                   
  pid_roll_setpoint /= 3.0;                                                 

  pid_pitch_setpoint = 0;
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;
  pid_pitch_setpoint -= pitch_level_adjust;                                  
  pid_pitch_setpoint /= 3.0; 

  pid_yaw_setpoint = 0;
  if(receiver_input_channel_3 > 1508)pid_yaw_setpoint = receiver_input_channel_3 - 1508;
  else if(receiver_input_channel_3 < 1492)pid_yaw_setpoint = receiver_input_channel_3 - 1492;                                  
  pid_pitch_setpoint /= 3.0; 

  calculate_pid();

  Serial.print("Receiver Roll: ");
  Serial.println(receiver_input_channel_1);
  Serial.print("Receiver Pitch: ");
  Serial.println(receiver_input_channel_2);
  Serial.print("Receiver Yaw: ");
  Serial.println(receiver_input_channel_3);
  Serial.print("Intensity Knob: ");
  Serial.println(receiver_input_channel_4);

  int servoVal1 = receiver_input_channel_1+pid_output_roll;
  int servoVal2 = receiver_input_channel_2+pid_output_pitch;
  //servoVal2=(1500-servoVal2)+1500;
  //int servoVal2 = receiver_input_channel_2-pid_output_roll;
  //int servoVal2 = (1500-servoVal1)+1500;
  int servoVal3 = servoVal1;
  servoVal3 = (1500-servoVal1)+1500;
  int servoVal4 = receiver_input_channel_3+pid_output_yaw;

  Serial.print("Calculated roll input : ");
  Serial.println(servoVal1);
  Serial.print("Calculated pitch input : ");
  Serial.println(servoVal2);
  Serial.print("Calculated Yaw input : ");
  Serial.println(servoVal4);

  while(micros() - loop_start_time < 4000);                                      
  loop_start_time = micros();                                                
  
  loopCounter++;
  
  if ( loopCounter >= 0 ) 
  {
  
    loopCounter = 0;
    
   PORTD|=B11110000 ;                                                                
    unsigned long timer_channel_1 = servoVal1 + loop_start_time;                  
    unsigned long timer_channel_2 = servoVal2 + loop_start_time;
    unsigned long timer_channel_3 = servoVal3 + loop_start_time;
    unsigned long timer_channel_4 = servoVal4 + loop_start_time;
                      
    byte cnt = 0;
    while(cnt < 4)
    {                                                                
      cnt = 0;
      unsigned long esc_loop_start_time = micros();                                
      if(timer_channel_1 <= esc_loop_start_time) {PORTD &= B11101111; cnt++;}              
      if(timer_channel_2 <= esc_loop_start_time) {PORTD &= B11011111; cnt++;}
      if(timer_channel_3 <= esc_loop_start_time) {PORTD &= B10111111; cnt++;}
      if(timer_channel_4 <= esc_loop_start_time) {PORTD &= B01111111; cnt++;}              
    }
  }
}

double mapf(double val, double in_min, double in_max, double out_min, double out_max) 
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup_mpu_6050_registers()
{
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x6B);                                                   
  Wire.write(0x00);                                                    
  Wire.endTransmission(); 
  delay(1000);                                             
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1C);                                                    
  Wire.write(0x10);                                                    
  Wire.endTransmission();
  delay(1000);                                              
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                    
  Wire.write(0x08);                                                    
  Wire.endTransmission();
  delay(1000);                                            
}

void read_mpu_6050_data()
{                                             
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                  
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,14);                                           
  while(Wire.available() < 14);                                      
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                 
  acc_z = Wire.read()<<8|Wire.read();                                 
  temperature = Wire.read()<<8|Wire.read();                         
  gyro_x = Wire.read()<<8|Wire.read();                               
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();  
  acc_x *= -1;
  gyro_y *= -1;                               

}

void calculate_pid()
{
  float pid_error_temp;
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  pid_i_mem_pitch = constrain(pid_i_mem_pitch, -pid_max_pitch, pid_max_pitch);
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  pid_output_pitch = constrain(pid_output_pitch, -pid_max_pitch, pid_max_pitch);
    
  pid_last_pitch_d_error = pid_error_temp;

  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  pid_i_mem_roll = constrain(pid_i_mem_roll, -pid_max_roll, pid_max_roll);
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  pid_output_roll = constrain(pid_output_roll, -pid_max_roll, pid_max_roll);
  
  pid_last_roll_d_error = pid_error_temp;

  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  pid_i_mem_yaw = constrain(pid_i_mem_yaw, -pid_max_yaw_i, pid_max_yaw_i);
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  pid_output_yaw = constrain(pid_output_yaw, -pid_max_yaw, pid_max_yaw);
    
  pid_last_yaw_d_error = pid_error_temp;
  
}

ISR(PCINT0_vect)
{
  
  if(last_channel_1 == 0 && PINB & B00000001 )
  {         
    last_channel_1 = 1;                                 
    timer_1 = micros();                                 
  }
  else if(last_channel_1 == 1 && !(PINB & B00000001))
  {  
    last_channel_1 = 0;                                 
    receiver_input_channel_1 = micros() - timer_1;      
  }
  
  if(last_channel_2 == 0 && PINB & B00000010 )
  {         
    last_channel_2 = 1;                                
    timer_2 = micros();                                 
  }
  else if(last_channel_2 == 1 && !(PINB & B00000010))
  {  
    last_channel_2 = 0;                                 
    receiver_input_channel_2 = micros() - timer_2;      
  }

  if(last_channel_3 == 0 && PINB & B00000100 )
  {         
    last_channel_3 = 1;                                 
    timer_3 = micros();                                 
  }
  else if(last_channel_3 == 1 && !(PINB & B00000100))
  {  
    last_channel_3 = 0;                                 
    receiver_input_channel_3 = micros() - timer_3;      
  }

  if(last_channel_4 == 0 && PINB & B00001000 )
  {         
    last_channel_4 = 1;                                 
    timer_4 = micros();                                 
  }
  else if(last_channel_4 == 1 && !(PINB & B00001000))
  {  
    last_channel_4 = 0;                                 
    receiver_input_channel_4 = micros() - timer_4;      
  }

}


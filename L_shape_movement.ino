void robot_heading(void);
void find_distance(void);
void stop_motors(void);
void take_turn();
void motors_start(int left, int left_dir, int right, int right_dir);


int left_encoder_pin = 2;
int left_motor_PWM_pin = 10;      
int left_motor_direction_pin = 5; 

int right_encoder_pin = 3;
int right_motor_PWM_pin = 9;       
int right_motor_direction_pin = 6;


           
int left_encoder_value  = 0;
int right_encoder_value = 0;

int left_encoder_angle_value  = 0;
int right_encoder_angle_value = 0;

int left_dist = 0;
int left_speed = 0;

int right_dist = 0;
int right_speed = 0;           



int theeta=90; //give angle of rotation  
int angle=(250/30)*theeta;



int distance_travel=20;



#define Reverse 0            
#define Forward 1              
#define Left    2             
#define Right   3

void setup()
{ Serial.begin(9600);    

  pinMode(left_encoder_pin , INPUT);
  pinMode(right_encoder_pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(left_encoder_pin),  find_distance, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder_pin), find_distance, CHANGE);

}



void loop()
{   motors(left_speed, Forward, right_speed, Forward); 
    while (left_dist >= distance_travel)
    {  
       attachInterrupt(digitalPinToInterrupt( left_encoder_pin), robot_heading, CHANGE);
       attachInterrupt(digitalPinToInterrupt(right_encoder_pin), robot_heading, CHANGE);     
       take_turn();
    }   
}



void motors(int left, int left_dir, int right, int right_dir)
{ analogWrite (right_motor_PWM_pin, right);          
  digitalWrite(right_motor_direction_pin, right_dir);
   
  analogWrite (left_motor_PWM_pin, left);             
  digitalWrite(left_motor_direction_pin, left_dir ); 
}



void stop_motors()
{  
    analogWrite(left_motor_PWM_pin ,255);    //right
    analogWrite(right_motor_PWM_pin,  255);  //left
}



void find_distance(void)
{ left_encoder_value++;
  if (left_encoder_value % 122 == 0)  //264=1 Meters in original mona motors 12172
  {  left_dist++;
     Serial.print("Left motor meters : "); 
     Serial.println(left_dist);
  }
  
  right_encoder_value++;              
  if (right_encoder_value % 122 == 0)     //if 1m is reached
  {  right_dist++;          
     Serial.print("Right motor meters : "); 
     Serial.println(right_dist);
  }
  
}



void take_turn() 
{
  analogWrite( left_motor_PWM_pin, 150);
  analogWrite(right_motor_PWM_pin,  55);

  digitalWrite( left_motor_direction_pin, Reverse);
  digitalWrite(right_motor_direction_pin, Forward);

}



void robot_heading()
{  left_encoder_angle_value++;
  right_encoder_angle_value++;

  while (left_encoder_angle_value && right_encoder_angle_value >= angle)
  {  
    analogWrite(left_motor_PWM_pin ,0);
    analogWrite(right_motor_PWM_pin,  255);
  }
  
}

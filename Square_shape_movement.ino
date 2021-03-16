void take_turn(void);
void take_turnoed(void);
void find_distance(void);
void move_straight(void);
void stop_linear_movement(void);
void stop_angular_movement(void);

int left_encoder_pin         =  2;
int left_motor_PWM_pin       = 10;      
int left_motor_direction_pin =  6; 

int right_encoder_pin         = 3;
int right_motor_PWM_pin       = 9;       
int right_motor_direction_pin = 5;
           
int left_encoder_value  = 0;
int right_encoder_value = 0;

int left_encoder_angle_value  = 0;
int right_encoder_angle_value = 0;

int left_dist  =  0;
int left_speed = 11;

int right_dist  = 0;
int right_speed = 0;           

int theeta = 90; //give angle of rotation  
int angle  = abs((theeta/30)*295);
 
int theeta2 = 180; //give angle of rotation  
int angle2  = (theeta2/30)*250;

int i       = 0;
int n       = 0;
int oed     = 0;
int linear  = 1;
int angular = 0;

int linear_distance = 20;
int square_complete =  0;

#define Reverse 0            
#define Forward 1              
#define Left    2             
#define Right   3

void setup()
{ Serial.begin(9600);
    
  pinMode(left_encoder_pin , INPUT);
  pinMode(right_encoder_pin, INPUT);
  
  attachInterrupt(digitalPinToInterrupt( left_encoder_pin), find_distance, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder_pin), find_distance, CHANGE);
}

void loop()
{  if (square_complete <= 4 )
   {   if (linear == 1)
       {  move_straight();  }
       else if (linear == 0) 
       {  take_turn();      }
       else if (oed==1) 
       {  take_turnoed();   }
   }
   else
   {   stop_linear_movement();   }
   
}

void move_straight()
{ analogWrite (right_motor_PWM_pin,   right_speed);          
  digitalWrite(right_motor_direction_pin, Forward);
  
  analogWrite (left_motor_PWM_pin,     left_speed);             
  digitalWrite(left_motor_direction_pin,  Forward); 
}

void find_distance(void)
{ 
  left_encoder_value++;
  right_encoder_value++;
  
  if (left_encoder_value  % 122 == 0) //264=1 Meters in original mona motors 12172
  {  left_dist++;   }           
  if (right_encoder_value % 122 == 0) //if 1m is reached
  {  right_dist++;  }


  if (left_dist >= linear_distance)
  {   left_encoder_value  = 0;
      right_encoder_value = 0;
      left_dist  = 0;
      right_dist = 0;
      
      linear     = 0;
      
      square_complete++; 
      angular++;
      
      if (angular == 4)  
      {   oed = 1;   }
  } 
  

  else if (linear==0)
  {    left_encoder_angle_value++;
       right_encoder_angle_value++;
      
      if (left_encoder_angle_value >= angle)
      {    left_encoder_angle_value=0;
           right_encoder_angle_value=0;
           linear=1; 
      }
  }
  else if (oed==1) 
  {   
      right_encoder_angle_value++;
      if (left_encoder_angle_value >= angle2)
      {     left_encoder_angle_value=0;
            right_encoder_angle_value=0;
            linear=1;
            oed=0;          
       } 
   }
}
void take_turn() 
{ digitalWrite( left_motor_direction_pin, Forward);
  digitalWrite(right_motor_direction_pin, Reverse); 
  
  analogWrite( left_motor_PWM_pin,  170);
  analogWrite(right_motor_PWM_pin,  30);
}

void take_turnoed(void)
{ //digitalWrite( left_motor_direction_pin, Reverse);
  digitalWrite(right_motor_direction_pin, Reverse); 
  
  //analogWrite( left_motor_PWM_pin,  0);
  analogWrite(right_motor_PWM_pin,  0);
}

void stop_linear_movement()
{  analogWrite(left_motor_PWM_pin , 255);    
   analogWrite(right_motor_PWM_pin, 255);     }
void stop_angular_movement()
{  analogWrite(left_motor_PWM_pin,   0);
   analogWrite(right_motor_PWM_pin,255);      }



   

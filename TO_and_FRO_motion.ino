void forward(void);
void reverse(void);
void find_distance(void);

int left_encoder_pin = 2;
int left_motor_PWM_pin = 10;      
int left_motor_direction_pin =6 ; 

int right_encoder_pin = 3;
int right_motor_PWM_pin = 9;       
int right_motor_direction_pin = 5;
           
int left_encoder_value  = 0;
int right_encoder_value = 0;

int left_dist = 0;
int left_speed = 0;

int right_dist = 0;
int right_speed = 8 ;           

int linear=1;
int linear_distance=20;

#define Reverse 0            
#define Forward 1              

void setup()
{ Serial.begin(9600);    
  pinMode(left_encoder_pin , INPUT);
  pinMode(right_encoder_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt( left_encoder_pin), find_distance, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder_pin), find_distance, CHANGE);
 
}

void loop()
{       forward();
        delay(1000);
        reverse();  
        delay(900);
   
          
}

void forward()
{ analogWrite (right_motor_PWM_pin, 55);          
  digitalWrite(right_motor_direction_pin, Forward);
  
  analogWrite (left_motor_PWM_pin, 55);             
  digitalWrite(left_motor_direction_pin, Forward );
   
}

void reverse()
{ analogWrite (right_motor_PWM_pin, 150);          
  digitalWrite(right_motor_direction_pin, Reverse);
  
  analogWrite (left_motor_PWM_pin, 150);             
  digitalWrite(left_motor_direction_pin,   Reverse);

}

 
void find_distance(void)
{   
  left_encoder_value++;
  right_encoder_value++;
  
  if (left_encoder_value % 122 == 0)  //264=1 Meters in original mona motors 12172
  {  left_dist++;    }           
  if (right_encoder_value % 122 == 0)     //if 1m is reached
  {  right_dist++;   }


  if (left_dist >= linear_distance)
  {   linear=0;  
  }

  if (linear==0)
  {    left_encoder_value++;
       right_encoder_value++;
      
      if (left_dist >= linear_distance)
      {    left_encoder_value=0;
           right_encoder_value=0;
           left_dist=0;
           right_dist=0;
           linear=1;  
      }
  }
 
}






   

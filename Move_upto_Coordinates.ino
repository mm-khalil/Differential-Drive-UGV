int left_pwm_pin=9;      //left motor pwm
int left_motor_direction_pin=6; //left motor pin, direction 0=FWD, 1=REV 
int left_encoder_interrupt_pin=2;
int left_dist_in_meters=0;
long int left_encoder_value=0;


int right_pwm_pin=10;     //right motor pwm
int right_motor_direction_pin=5; // motor direc tion we give on pin 5, direction 0=FWD, 1=REV 
int right_encoder_interrupt_pin=3;
int right_dist_in_meters=0;
long int right_encoder_value=0;

int i=0,j=0,k=0;

long int x_coordinate=10;
long int y_coordinate=15;
long int yaw=10;
#define Forward 1
#define Reverse 0

void setup()
{  Serial.begin(9600);
   pinMode(left_encoder_interrupt_pin, INPUT); 
   //pinMode(right_encoder_interrupt_pin,INPUT);   
}
void loop()
{  while (k!=yaw)  
   {      right_motor_fun();
        
   }

   
   
//  while(i!=x_coordinate && j!=y_coordinate)
//    {   left_motor_fun();
//        right_motor_fun();
//        i++;
//        j++;   
//    }  
}
void right_motor_fun()
{  digitalWrite(right_pwm_pin,1);
   digitalWrite(right_motor_direction_pin,0);   
   attachInterrupt(digitalPinToInterrupt(right_encoder_interrupt_pin), find_right_dist, CHANGE);
}
void find_right_dist(void)
{  right_encoder_value++;
   /*if (right_encoder_value == 264)
//   {   digitalWrite(right_pwm_pin,0);
//       right_encoder_value=0;
//       right_dist_in_meters++;  }*/
}
//void left_motor_fun()
//{  digitalWrite(left_pwm_pin,1);
//   digitalWrite(left_motor_dir ection_pin,0);
//   attachInterrupt(digitalPinToInterrupt(left_encoder_interrupt_pin),find_left_dist,CHANGE);
//}
//void find_left_dist(void)
//{  left_encoder_value++;
//   /*if (left_encoder_value == 264)  //264=1 Meters in original mona motors
//   {   digitalWrite(left_pwm_pin,0);
//       left_encoder_value=0;
//       left_dist_in_meters++;   } */ 
//}
 void rotation()
{  digitalWrite(left_pwm_pin,0);
   digitalWrite(left_motor_direction_pin,Forward);
    digitalWrite(right_pwm_pin,255);
   digitalWrite(right_motor_direction_pin,Reverse); 
    k++;
}

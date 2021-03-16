void rotation(long int angle);

long int theeta1=90;  //angle is in degrees
long int angle1=(250/30)*theeta1;//for 90degrees we must read encoder tips which are =750, so write 750 instead of 90in theeta

int left_encoder_interrupt_pin =2;
int right_encoder_interrupt_pin=3;

int left_encoder_value=0;
int right_encoder_value=0;

int left_motor_direction_pin=6;
int right_motor_direction_pin=5;

int left_pwm_pin=10;
int right_pwm_pin=9;

#define Reverse 0            
#define Forward 1

void setup()
{  Serial.begin(9600);

   pinMode(left_encoder_interrupt_pin, INPUT); 
   pinMode(right_encoder_interrupt_pin,INPUT);

   pinMode(left_pwm_pin, OUTPUT);
   pinMode(right_pwm_pin,OUTPUT);

   attachInterrupt(digitalPinToInterrupt(left_encoder_interrupt_pin ),rotation,CHANGE);
   attachInterrupt(digitalPinToInterrupt(right_encoder_interrupt_pin),rotation,CHANGE);
}

void loop() {
   motors_start();


}
void motors_start()
{  
   analogWrite(left_pwm_pin,150);
   analogWrite(right_pwm_pin,55); 

   digitalWrite( left_motor_direction_pin,Forward);
   digitalWrite(right_motor_direction_pin,Reverse);

}

void rotation(long int angle1)
{  left_encoder_value++;
   right_encoder_value++;
   
   while (left_encoder_value &&  right_encoder_value >= angle1)
   {   analogWrite(left_pwm_pin,255);   
       analogWrite( right_pwm_pin,0);   }    
}

  

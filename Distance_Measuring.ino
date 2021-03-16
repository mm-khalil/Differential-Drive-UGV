//Left motor
int left_pwm=9;      //left motor pwm
int left_motor=6; //left motor pin, direction 0=FWD, 1=REV 
long int left_encoder_value=0;  //encoder value store in it
int left_encoder=2; //left encoder pin
/*right motor
int right_pwm=10;     //left motor pwm
int right_motor=5; // motor pin, direction 0=FWD, 1=REV 
long int right_encoder_value=0;  //encoder value store in it
int right_encoder=3; // encoder pin
*/
void setup()
{  Serial.begin(9600);
   pinMode(left_encoder,INPUT); 
  // pinMode(right_encoder,INPUT);   
}
void loop()
{  
    left_motorf();
    //right_motorf();
   
}
void left_count ()
{  while(left_encoder_value!=264)
  { left_encoder_value++;
   Serial.print("left encoder value = ");
   Serial.println(left_encoder_value);
   digitalWrite(left_pwm,1);  }
    /*
   if (left_encoder_value > 264)  //264=1 Meters in original mona motors
   {  digitalWrite(left_pwm,0);
      Serial.println("264 so stops left");
      delay(5000); 
      left_encoder_value=0;
      Serial.print("after 264 LEV = ");
      Serial.println(left_encoder_value);
      delay(5000);
   }*/
}
/*
void right_count ()
{  right_encoder_value++;
   Serial.print("right_encoder_value = ");
   Serial.println(right_encoder_value);
   
   if (right_encoder_value == 264)
   { digitalWrite(right_pwm,0);
     Serial.println("264 so stops right");
     delay(5000);
     right_encoder_value=0;  
   }
}
*/
void left_motorf()
{  digitalWrite(left_pwm,1);
   digitalWrite(left_motor,0);
   attachInterrupt(digitalPinToInterrupt(left_encoder),left_count,CHANGE);
   //digitalRead(left_encoder);
   //Serial.println("left motor works ");
   //Serial.println(left_encoder);    
}
/*
void right_motorf()
{  digitalWrite(right_pwm,1);
   digitalWrite(right_motor,0);   
   attachInterrupt(digitalPinToInterrupt(right_encoder), right_count, CHANGE);
}
*/

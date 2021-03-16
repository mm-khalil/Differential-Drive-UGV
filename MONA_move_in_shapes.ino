#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

int Motor_right_PWM = 10;  //   0 (min speed) - 255 (max speed) 
int Motor_right_direction = 5;  //   0 Forward - 1 Reverse
int Motor_left_PWM = 9;    //   0 (min speed) - 255 (max speed)  
int Motor_left_direction = 6;   //   0 Forward - 1 Reverse
 
#define Forward 1
#define Reverse 0  



/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
 
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel=0, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
   // Serial.print("! ");
  }

 
}


void setup() 
{
  pinMode(Motor_left_PWM, OUTPUT);
  pinMode(Motor_right_PWM, OUTPUT);


   Serial.begin(115200);
  //Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

   
}

void loop()
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  
  /*Serial.print("Yaw = : ");
  Serial.println(event.orientation.x, 4);*/
 
  Serial.print("x = : ");
  Serial.println((float)event.acceleration.x);
 /* Serial.print("Y = : ");
  Serial.println((float)event.acceleration.y);
  Serial.print("Z = : ");
  Serial.println((float)event.acceleration.z);
  */


  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

  straight();
  if (event.acceleration.x>350 ){
  stopmona();
  Serial.println("x= 40, so mona stops ");}
} 


void straight()
{
  analogWrite (Motor_right_PWM,40); 
  digitalWrite(Motor_right_direction,Forward); 
  analogWrite (Motor_left_PWM, 40);  
  digitalWrite(Motor_left_direction, Forward); 
}


void stopmona()
{
  analogWrite (Motor_right_PWM,0); 
  digitalWrite(Motor_right_direction,Forward); 
  analogWrite (Motor_left_PWM, 0);  
  digitalWrite(Motor_left_direction, Forward); 
}















 

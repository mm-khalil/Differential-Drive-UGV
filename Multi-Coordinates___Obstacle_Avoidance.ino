//----------------------***SIMBOT***----------------------//
//--------------------------------------------------------//
//------------ Multi-Coordinates-Code**START** -----------//
//--------------------------------------------------------//
// PWM Pins for
int R_PWM = 10;
int L_PWM = 9;
 
//  Motor Direction Pins
int R_direction = 5;
int L_direction = 6;
#define Forward 1
#define Reverse 0

//  Motor Encoder Pins
int R_encoder = 2;
int L_encoder = 3;
long int encoderLPos = 0;
long int encoderRPos = 0;

// Simbot Design Features
float dia = 0.034; // Wheel Diameter in m
float ER  = 1350;  // Wheel Revolotions per meter
float b   = 0.084; // Distance between both wheels

// Universal Frame Grid size
float grid = 0.10;

// Transformation Variables
#include <MatrixMath.h>
#define N  (3)
#define M  (1)
mtx_type T[N][N];   // Transformation Matrix
mtx_type Up[N][M];  // Target Point in Universal Frame
mtx_type Rp[N][M];  // Target Point in Robot Frame

// Robot Rotation and Translation in Universal Frame
float th = 0;    // Robot rotation in universal Frame
float th_rad = 0;    // Robot rotation in universal Frame

float Tx = 0;    // Robot Translation in Universal Frame (Position in Universal Frame)
float Ty = 0;

// Target Point In Universal Frame
float Ux = 0;
float Uy = 0;

// Next Point In Robot Frame
float Rx = 0;
float Ry = 0;

// Action parameters(Angle and Distance)to reach target coordinate
float Angle  = 0;
float A_Dist = 0; // Angular distance of wheels to rotate at a particular angle
float Dist   = 0; // Linear distance of wheels to reach Target coordinate
float A_360  = 360.0;


// Flags for Forward, CW and CCW Rotation
bool F, L, R;

// Rotation Variable
float Dr = 0;
float Dl = 0;
float Dc = 0;

// Calibration
int nl = 0;    //no of left turns
int nr = 0;    //no. of right turns
int last_action = 0;    // -1 for left,   1 for right

// Rotation
float l_tol  = 0;         
float fl_tol = 0;
float ll_tol = 0.01;
float rl_tol = 0.03;

float r_tol  = 0;
float fr_tol = 0;
float rr_tol = 0;
float lr_tol = 0;

// Forward Speed
int FW_R = 30;           // Forward (max 0 - min 255)
int FW_L = 40;           // Forward (max 0 - min 255)

// Left Speed
int CCW_R = 180;         // Forward (max 0 - min 255)
int CCW_L = 40;          // Reverse (max 255 - min 0)

// Right Speed
int CW_R = 40;           // Forward (max 0 - min 255)
int CW_L = 180;          // Reverse (max 255 - min 0)

// Target Coordinates in Universal Frame (Loading Mission coordinates)
int tar_x[] = {0,1,2};
int tar_y[] = {0,0,0};
int n = 0; // No. of Target Points
int p = 0; // Current Target Point number





#include <MsTimer2.h>
// pin config for basic platform test
// Motors 
int Motor_right_PWM       = 10;  //   0 (min speed) - 255 (max speed) 
int Motor_right_direction = 5;  //   0 Forward - 1 Reverse
int Motor_left_PWM        = 9;    //   0 (min speed) - 255 (max speed)  
int Motor_left_direction  = 6;   //   0 Forward - 1 Reverse

// IR
int IR_enable = 4;
int IR_threshold= 600; // 0 white close obstacle -- 1023 no obstacle

const byte Left_interruptPin  = 2;
const byte Right_interruptPin = 3;

int Left_forward_speed  = 111;
int Right_forward_speed = 111;
float slope             = 0.1;

int obstacle = 0;



void setup() {
    //-------Multi coordinates-------//

  Serial.begin (9600);
  // Taking Inpute from Encoders
  pinMode(R_encoder, INPUT);
  pinMode(L_encoder, INPUT);

  // Counting revolutions of right and left wheel
  attachInterrupt(digitalPinToInterrupt(R_encoder), doEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_encoder), doEncoder, CHANGE);
    





//-------obstacle avoidance-------//
  pinMode(Motor_left_PWM,  OUTPUT);
  pinMode(Motor_right_PWM, OUTPUT);

  pinMode(IR_enable, OUTPUT);
  // set encoder counter to 0
  int Left_counter  = 0;
  int Right_counter = 0;

  // init INT0 and INT1 for left and right motors encoders 
  pinMode(Left_interruptPin,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Left_interruptPin), Left_int_counter, CHANGE);
  pinMode(Right_interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Right_interruptPin), Right_int_counter, CHANGE);
  // setup Timer2 for motor control
  MsTimer2::set(400, Timer_overflow); // 400ms period
  MsTimer2::start();
  //set PWM set-point
  forwardoa();





}

void loop() {
  
 //-------obstacle avoidance-------//
 IR_proximity_read();
// if (obstacle == 1){ 
//     stopoa();    
//     Obstacle_avoidance();
// }
 /*
 else { 
  //-------Multi coordinates-------//
  n = sizeof(tar_x) / sizeof(int);
  if (p < n - 1) {
    /*Serial.print("point ");
    Serial.print(tar_x[p]);
    Serial.print(", ");
    Serial.print(tar_y[p]);
    
    Serial.print(" to point ");
    Serial.print(tar_x[p+1]);
    Serial.print(", ");
    Serial.println(tar_y[p+1]);


    calculate_action();
    update_heading();
    perform_action();
    p++;
  }
}*/
}


// Simbot Functions
void doEncoder() {

  encoderLPos++;
  encoderRPos++;
  //  if (L==1){
  //  encoderLPos--;
  //  encoderRPos++;
  //  }
  //  if (R==1){
  //  encoderLPos++;
  //  encoderRPos--;
  //  }
  //  if (F==1){
  //  encoderLPos++;
  //  encoderRPos++;
  //  }

}


void CCW() {   //left +ve angle

  Dr = 3.14 * 0.034 * (encoderRPos / ER);
  Dl = 3.14 * 0.034 * (encoderLPos / ER);
  Dc = (Dr + Dl) / 2;

  if (Dc < A_Dist + l_tol)

  {
    analogWrite(R_PWM, CCW_R);
    analogWrite(L_PWM, CCW_L);
    digitalWrite(R_direction, Forward);
    digitalWrite(L_direction, Reverse);
  }

  if (Dc >= A_Dist + l_tol)
  {
    analogWrite(R_PWM, 255);
    analogWrite(L_PWM, 0);
    L = 0;
  }
}

void CW() { //Right -ve angle

  Dr = 3.14 * 0.034 * (encoderRPos / ER);
  Dl = 3.14 * 0.034 * (encoderLPos / ER);
  Dc = (Dr + Dl) / 2;

  if (Dc < A_Dist + r_tol)

  {
    analogWrite(R_PWM, CW_R);
    analogWrite(L_PWM, CW_L);
    digitalWrite(R_direction, Reverse);
    digitalWrite(L_direction, Forward);
  }

  if (Dc >= A_Dist + r_tol)
  {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 255);
    R = 0;
  }
}

void forward() {
  Dr = 3.14 * 0.034 * (encoderRPos / ER);
  Dl = 3.14 * 0.034 * (encoderLPos / ER);
  Dc = (Dr + Dl) / 2;
  if (Dc < grid * Dist)
  {
    analogWrite(R_PWM, FW_R);
    digitalWrite(R_direction, Forward);
    analogWrite(L_PWM, FW_L);
    digitalWrite(L_direction, Forward);
  }
  if (Dc >= grid * Dist)
  {
    analogWrite(R_PWM, 255);
    analogWrite(L_PWM, 255);
    F = 0;
  }
}

void calculate_action() {



  Serial.print("Robot Heading : ");
  Serial.println(th);
  Serial.print("Last Action : ");
  Serial.println (last_action);

  // Position of robot in Universal Frame
  Tx = tar_x[p];
  Ty = tar_y[p];
  // Target Point in Universal Frame
  Ux = tar_x[p + 1];
  Uy = tar_y[p + 1];

  //Transformation Matrix
  th_rad = th * 3.14 / 180;
  T[0][0] = cos(th_rad);
  T[0][1] = -sin(th_rad);
  T[0][2] = Tx;
  T[1][0] = sin(th_rad);
  T[1][1] = cos(th_rad);
  T[1][2] = Ty;
  T[2][0] = 0;
  T[2][1] = 0;
  T[2][2] = 1;

  // Target Point in Universal Frame
  Up[0][0] = Ux;
  Up[1][0] = Uy;
  Up[2][0] = 1;


  // Finding Target points in Robot Frame
  Matrix.Invert((mtx_type*)T, N);
  // Serial.println("\nInverted T:");
  // Matrix.Print((mtx_type*)A, N, N, "A");
  Matrix.Multiply((mtx_type*)T, (mtx_type*)Up, N, N, M, (mtx_type*)Rp);
  // Serial.println("\nAfter multiplying C = A*B:");
  // Matrix.Print((mtx_type*)T, N, N, "T");
  // Matrix.Print((mtx_type*)Up, N, M, "Up");
  // Matrix.Print((mtx_type*)Rp, N, M, "Rp");

  // Target Point in Robot Frame
  Rx = Rp[0][0];
  Ry = Rp[1][0];

  // Rotation required by Robot to reach Target Point and the Angular Distance to create that angle
  Angle = (atan2(Ry, Rx)) * (180 / 3.14);
  Serial.print("Angle : ");
  Serial.println(Angle);
  A_Dist = abs((Angle / 360.0) * 3.14 * b);
  Serial.print("Angular Distance : ");
  Serial.println(A_Dist);

  // Distance required by Robot to reach Target Point
  Dist = sqrt(pow(Ry, 2) + pow(Rx, 2));
  Serial.print("Distance : ");
  Serial.println(Dist);
    if (last_action<0)
  {
    r_tol=lr_tol;
    l_tol=ll_tol;
  }
  else if (last_action>0)
  {
     l_tol=rl_tol;  
     r_tol=rr_tol;
  }
  else 
  {
    l_tol=fl_tol;
    r_tol=fr_tol;

  }
}

void update_heading() {
  // Update Robot Orientation
  th = (Angle + th);
  if (th >= 360) {
    th = th - 360;          // to define the range as 0 to 360
  }
  else if (th <= -360) {
    th = th + 360;          // to define the range as 0 to -360
  }
  //  Serial.print("Robot theta After Action: ");
  //  Serial.println(th);
}

void perform_action() {
  Serial.print("Tolerance : ");
  Serial.println(l_tol);
  if (Angle > 0) {
    L = 1;
    R = 0;
    F = 0;
    last_action = -1;
    Serial.println("Rotate Left");

    nl++;
  }
  else if (Angle < -0.1) {
    L = 0;
    R = 1;
    F = 0;
    Serial.println("Rotate Right");
    last_action = 1;
    nr++;
  }
  else {
    L = 0;
    R = 0;
    F = 0;
    Serial.println("No Rotation");

  }
  if (nl % 4 == 0)
  {
    l_tol = l_tol + (0.0015);
  }
  if (nr % 4 == 0)
  {
    r_tol = r_tol + (0.0015);
  }


  while (L != 0)
  {
    CCW();
  }
  while (R != 0)
  {
    CW();
  }
  encoderLPos = 0;
  encoderRPos = 0;
  delay(1000);

  if (Dist > 0) {
    L = 0;
    R = 0;
    F = 1;
    Serial.println("Move Forward");
  }
  while (F != 0)
  {
    forward();
  }
  encoderLPos = 0;
  encoderRPos = 0;
  delay(1000);
  Serial.println(" ");
}


//--------------------------------------------------------//
//------------- Multi-Coordinates-Code**END** ------------//
//--------------------------------------------------------//


//--------------------------------------------------------//
//----------- Obstaclde-Avoidance-Code**START** ----------//
//--------------------------------------------------------//
//Motion Control for Open-loop and Close-loop 

void forwardoa(){
  analogWrite(Motor_right_PWM,Right_forward_speed); // right motor
  digitalWrite(Motor_right_direction,Forward); //right
  analogWrite(Motor_left_PWM,Left_forward_speed); // left motor
  digitalWrite(Motor_left_direction,Forward); //left
}
                                                                                              
void Stopoa(){ // set speeds to 0
  analogWrite(Motor_right_PWM,255); // right motor
  analogWrite(Motor_left_PWM,255  );  // left 
  MsTimer2::start();
}  

// Variables for 5 IR proximity sensors 
int IR_right,IR_right_front,IR_front,IR_left_front,IR_left;

void IR_proximity_read(){    // read only front IR sensor
  int n=5;  // average parameter
  digitalWrite(IR_enable, HIGH);  //IR Enable
  IR_right=0;
  IR_right_front=0;
  IR_front=0;  
  IR_left_front=0;
  IR_left=0;
  for (int i=0;i<n;i++){  //read from front proximity sensor 
    IR_right+=analogRead(A3);
    IR_right_front+=analogRead(A2);
    IR_front+=analogRead(A1);
    IR_left_front+=analogRead(A0);
    IR_left+=analogRead(A7);
    delay(20);
if (IR_right>0){
   Stopoa();
}
    
  }
  IR_right/=n;
  IR_right_front/=n;
  IR_front/=n;
  IR_left_front/=n;
  IR_left/=n;
    
}

void Obstacle_avoidance(){
  if (IR_front<IR_threshold || IR_right<IR_threshold || IR_right_front<IR_threshold || IR_left<IR_threshold || IR_left_front<IR_threshold){
//       digitalWrite(LED1,HIGH);
//Serial.println("oa stop");
//      Stopoa();
     
  }  
}

long int Left_counter, Right_counter;

void Left_int_counter(){
  Left_counter++;
}

void Right_int_counter(){
  Right_counter++;
}

float Left_compensatory=1,Right_compensatory=1;
int Desired = 910;   // number of pulses which is desired
int run_exp=0;

void Timer_overflow() {   // every 400 ms is called
  run_exp++;
  Right_compensatory =  Desired - Right_counter;
  Left_compensatory  =  Desired - Left_counter;  
  /*
  Serial.print("Left = ");
  Serial.print(Left_counter);
  Serial.print(" , ");
  Serial.print("  Right = ");
  Serial.println(Right_counter);
*/
  // a basic proportional control (disable these two lines for Open-loop control)
  Right_forward_speed +=  slope * Right_compensatory;
  Left_forward_speed  +=  slope *  Left_compensatory;

  forwardoa();  // update PWM set-point 

  //reset counters 
  Left_counter = 0;
  Right_counter= 0;
}

//--------------------------------------------------------//
//------------ Obstaclde-Avoidance-Code**END** -----------//
//--------------------------------------------------------//

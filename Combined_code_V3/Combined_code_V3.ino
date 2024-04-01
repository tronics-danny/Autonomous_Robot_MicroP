#include<Servo.h>
#include<util/atomic.h>

//Encoder outputs for motor A connections
#define MotorA_ENCA 18
#define MotorA_ENCB 19
//Encoder outputs for motor B connections
#define MotorB_ENCA 2
#define MotorB_ENCB 3

//Defining Servo motor and ultrasonic sensor connected pins
#define C_TrigPin 33 //Centre Ultrasonic sensor
#define C_EchoPin 35
#define L_TrigPin 38 //Left Ultrasonic sensor
#define L_EchoPin 39
#define R_TrigPin 36 //Right Ultrasonic sensor
#define R_EchoPin 37
#define ServoPin 10

//defining pins for LEDs
int Green_led_R = 32;
int Green_led_L = 28;
int Red_led = 30;
int Yellow_led = 34;

//Motor driver connections for motor A
int enA = 4; //PWM Signal for motor A
int en1 = 5;
int en2 = 6;
//Motor driver connections for motor B
int enB = 9; //PWM Signal for motor B
int en3 = 7;
int en4 = 8;

//Defining global storage variables that can be used to store values between time steps
long prevT = 0; //previous time
int posPrevA = 0; //previous position for motor A
int posPrevB = 0; //previous position for motor B

long currT = 0;
float deltaT = 0; 

//Gloabal variables to store the number of counts of the encoders
volatile int pos_i_A = 0;
volatile int pos_i_B = 0;

volatile float velocity_i_A = 0;
volatile float velocity_i_B = 0;
volatile long prevT_i = 0;

//Defining global variables for the Raw and Filtered velocities
float v1FiltA = 0;
float v1PrevA = 0;

float v2FiltB = 0;
float v2PrevB = 0;

//Global variable to store the integral error value
float error1_integral = 0;
float error2_integral = 0;


// Defining variables to store the ultrasonic sensor distances 
float C_distance = 0;
float L_distance = 0;
float R_distance = 0;

// Defining min distances to be detected
float C_distance_Min = 15; //min distance for the centre ultrasonic sensors
float S_distance_Min = 15; //min distance for the side's ultasonic sensors


//Defining the constant target speed of every motor
float targetSpeedA = 100;
float targetSpeedB = 100;

//global variables to store the calculated rpm speeds
float v1 = 0;
float v2 = 0;


void setup() {
  Serial.begin(9600);

  //Labels for the serial plotter motor encoder value outputs
  Serial.println("Motor_A_Speed, Motor_B_Speed");

  //setting ultrasonic sensor pins
  pinMode(C_TrigPin, OUTPUT);
  pinMode(C_EchoPin, INPUT);
  pinMode(L_TrigPin, OUTPUT);
  pinMode(L_EchoPin, INPUT);
  pinMode(R_TrigPin, OUTPUT);
  pinMode(R_EchoPin, INPUT);

  //Setting Led pins
  pinMode(Green_led_R, OUTPUT);
  pinMode(Green_led_L, OUTPUT);
  pinMode(Red_led, OUTPUT);
  pinMode(Yellow_led, OUTPUT);

  pinMode(ServoPin, OUTPUT);

  //setting up encoder pins for the motors as inputs to the microcontroller
  pinMode(MotorA_ENCA,INPUT);
  pinMode(MotorA_ENCB,INPUT);
  pinMode(MotorB_ENCA,INPUT);
  pinMode(MotorB_ENCB,INPUT); 
  //Setting up pins connected to motor driver as outputs
  pinMode(enA,OUTPUT);
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
  pinMode(en3,OUTPUT);
  pinMode(en4,OUTPUT);
  pinMode(enB,OUTPUT);

  //Setting encoder output for motor A and B ENCA to trigger an interrupt then call a function
  attachInterrupt(digitalPinToInterrupt(MotorA_ENCA),readEncoderA,RISING);
  attachInterrupt(digitalPinToInterrupt(MotorB_ENCA),readEncoderB,RISING);

}

void loop() {
  //calling the read Utrasonic function
  float velocityA = 0;
  float velocityB = 0;
  int pos1 = 0;
  int pos2 = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos1 = pos_i_A;
    pos2 = pos_i_B;
    velocityA = -velocity_i_A;
    velocityB = velocity_i_B;
  }

  //compute velocity with method of counting pulses per time interval
  long currT = micros();
  float deltaT = ((float) (currT-prevT)) / 1.0e6;
  float velocity1 = -((pos1 - posPrevA) / deltaT);
  posPrevA = pos1;
  float velocity2 = (pos2 - posPrevB) / deltaT;
  posPrevB = pos2;
  prevT = currT;

  //Convert the velocities to RPM
  v1 = velocity1/362.0*60.0;
  v2 = velocity2/362.0*60.0;

  //LowPass Filter for the velocities at (25 Hz Cut off frequency)
  v1FiltA = 0.854*v1FiltA + 0.0728*v1 + 0.0728*v1PrevA;
  v1PrevA = v1;
  v2FiltB = 0.854*v2FiltB + 0.0728*v2 + 0.0728*v2PrevB;
  v2PrevB = v2;

  //a function called that will drive the two motors at 100rpm
  //compareRPM(50, 50, v1, v2, 1, 1);
  
  //Drive the robot
  compare_distances();

  //Print the velocities on the serial monitor
  //Serial.print(velocity1);
  //Serial.print(" ");
  Serial.print(v1);
  Serial.print(" ");
  //Serial.print(velocity2);
  //Serial.print(" ");
  Serial.print(v2);
  //Serial.print(" ");
  //Serial.print(velocityA);
  //Serial.print(" ");
  //Serial.print(velocityB);
  Serial.println();

  delay(1);


}

void compareRPM(int t_SpeedA, int t_SpeedB, int rpm1, int rpm2, int dirA, int dirB){
  //compute the control signal u
  float kp1 = 11;
  float ki1 = 0.5;
  float kp2 = 8.7;
  float ki2 = 0.7;

  float error1 = t_SpeedA - rpm1;
  float error2 = t_SpeedB - rpm2;

  //Computing the integral error
  error1_integral = error1_integral + error1*deltaT;
  error2_integral = error2_integral + error2*deltaT;

  //Computing the control signal
  float u1 = kp1*error1 + ki1*error1_integral;
  float u2 = kp2*error2 + ki2*error2_integral;

  //set the motor direction and speed
  if (u1<0){
    dirA = -1;
  }
  if (u2<0){
    dirB = -1;
  }

  int pwmValA = (int) fabs(u1);
  int pwmValB = (int) fabs(u2);
  if (pwmValA > 255){
    pwmValA = 255;
  }
  if (pwmValB > 255){
    pwmValB = 255;
  }
  //calling the two motors to move in a forward direction at the target speed
  setMotorA(dirA, pwmValA);
  setMotorB(dirB, pwmValB);  

}

void setMotorA(int dirA, int pwmValA){
  analogWrite(enA, pwmValA);

  if (dirA == 1){
    //MotorA to rotate in forward direction
    digitalWrite(en1, HIGH);
    digitalWrite(en2, LOW);
  }
  else if (dirA == -1){
    //MotorA to rotate in reverse direction 
    digitalWrite(en1, LOW);
    digitalWrite(en2, HIGH);
  }
  else{
    //Motor to stop
    digitalWrite(en1, LOW);
    digitalWrite(en2, LOW);
  }
}

void setMotorB(int dirB, int pwmValB){
  analogWrite(enB, pwmValB);

  if (dirB==1){
    //Motor 2 to rotate in forward direction
    digitalWrite(en3, LOW);
    digitalWrite(en4, HIGH);
  }
  else if (dirB==-1){
    //Motor to rotate in reverse direction 
    digitalWrite(en3, HIGH);
    digitalWrite(en4, LOW);
  }
  else{
    //Motor to stop
    digitalWrite(en3, LOW);
    digitalWrite(en4, LOW);
  }
}

void readEncoderA(){
  //Reading from the encoder output B
  int a = digitalRead(MotorA_ENCB);
  int incrementA = 0;
  //If function to icrement position value for forward direction and decrement position value for reverse rotation of the motorA
  if (a>0){
    incrementA = 1;
  }
  else{
    incrementA = -1;
  }
  pos_i_A = pos_i_A + incrementA;

  //Compute velocity with method of measuring the time elapsed between triggers
  long currT = micros();
  float deltaT = ((float) (currT-prevT_i)) / 1.0e6;
  velocity_i_A = incrementA/deltaT;
  prevT_i = currT;

}

void readEncoderB(){
  //Reading from the encoder output B
  int b = digitalRead(MotorB_ENCB);
  int incrementB = 0;
  //If function to icrement position value for forward direction and decrement position value for reverse rotation of the motorA
  if (b>0){
    incrementB = 1;
  }
  else{
    incrementB = -1;
  }
  pos_i_B = pos_i_B + incrementB;

  //Compute velocity with method of measuring the time elapsed between triggers
  long currT = micros();
  float deltaT = ((float) (currT-prevT_i)) / 1.0e6;
  velocity_i_B = incrementB/deltaT;
  prevT_i = currT;

}

int UltraRead_C(){
  //Read detected distance by the three ultrasonic sensors
  //Triger the ultrasonic sensor low
  digitalWrite(C_TrigPin, LOW); 
  delayMicroseconds(2);
  //Triger the ultrasonic sensor High and delay for 10 micro-seconds then trig low
  digitalWrite(C_TrigPin, HIGH);
  delayMicroseconds(10);
  //Calculate distance from object
  long C_duration = pulseIn(C_EchoPin, HIGH);
  // Expression to calculate distance using time
  C_distance = C_duration * 0.0344 / 2; 

  if (C_distance < 15) { 
        digitalWrite (Green_led_R, LOW);
        digitalWrite (Green_led_L, LOW);
        delay(100);
        digitalWrite (Green_led_L, HIGH);
        digitalWrite (Green_led_R, HIGH);
        delay(50);
      }
    else {
        digitalWrite (Green_led_R, HIGH);
        digitalWrite (Green_led_L, HIGH);
      }

  return C_distance;
  
}

int UltraRead_R(){
  //Read detected distance by the three ultrasonic sensors
  //Triger the ultrasonic sensor low
  digitalWrite(R_TrigPin, LOW); 
  delayMicroseconds(2);
  //Triger the ultrasonic sensor High and delay for 10 micro-seconds then trig low
  digitalWrite(R_TrigPin, HIGH);
  delayMicroseconds(10);
  //Calculate distance from object
  long R_duration = pulseIn(R_EchoPin, HIGH);
  // Expression to calculate distance using time
  R_distance = R_duration * 0.0344 / 2; 

    if (R_distance < 15) { 
        digitalWrite (Green_led_R, LOW);
        delay(100);
        digitalWrite (Green_led_R, HIGH);
        delay(50);
      }
    else {
        digitalWrite (Green_led_R, HIGH);
      }

  return R_distance;
  
}

int UltraRead_L(){
  //Read detected distance by the three ultrasonic sensors
  //Triger the ultrasonic sensor low
  digitalWrite(L_TrigPin, LOW); 
  delayMicroseconds(2);
  //Triger the ultrasonic sensor High and delay for 10 micro-seconds then trig low
  digitalWrite(L_TrigPin, HIGH);
  delayMicroseconds(10);
  //Calculate distance from object
  long L_duration = pulseIn(L_EchoPin, HIGH);
  // Expression to calculate distance using time
  L_distance = L_duration * 0.0344 / 2;

    //Blinking the LEDs to indicate detection
    if (L_distance < 15) { 
        digitalWrite (Green_led_L, LOW);
        delay(100);
        digitalWrite (Green_led_L, HIGH);
        delay(50);
      }
    else {
        digitalWrite (Green_led_L, HIGH);
      }

  return L_distance;
  
}

void compare_distances(){
  //call distance measuring functions
  C_distance = UltraRead_C();
  R_distance = UltraRead_R();
  L_distance = UltraRead_L();


  if (C_distance > C_distance_Min && L_distance > S_distance_Min && R_distance > S_distance_Min){
    //If there is no obstacle on any side, then move straight forward
      compareRPM(70, 70, v1, v2, 1, 1);
   }
  else if (C_distance < C_distance_Min && L_distance < S_distance_Min && R_distance < S_distance_Min){ 
    //If there is an obstacle on the three sides, move back, compare left and right distances, select which distance to move into
      compareRPM(0, 0, v1, v2, 0, 0); //stop
      delay(200);
      compareRPM(50, 50, v1, v2, -1, -1); //move back
      delay(200);
      compareRPM(0, 0, v1, v2, 0, 0); //stop
      delay(100);

      //call distance measuring functions
        R_distance = UltraRead_R();
        L_distance = UltraRead_L();

      if (L_distance > S_distance_Min && L_distance > R_distance){
        //if distance on the left is greater than distance on the right, then move left
          compareRPM(20, 50, v1, v2, 1, 1); //move left
          delay(200);
          //compare_distances();
      }
      else if (R_distance > S_distance_Min && R_distance > L_distance){
        //if distance on the right is greater than distance on the left, then move right
          compareRPM(50, 20, v1, v2, 1, 1); //move left
          delay(200);
          //compare_distances();
      }
      else{
        //rotate for to face back
          compareRPM(20, 20, v1, v2, -1, 1); 
          delay(200);
          compareRPM(0, 0, v1, v2, 0, 0); //stop
          delay(100);
          //compare_distances();
      }
 
  }
  else if (C_distance > C_distance_Min && L_distance > S_distance_Min && R_distance < S_distance_Min){
    //If there is an obstacle on right side, then move slight  left
      compareRPM(50, 70, v1, v2, 1, 1);
      delay(200);
      //compare_distances();
  }
  else if (C_distance > C_distance_Min && R_distance > S_distance_Min && L_distance < S_distance_Min){
    //If there is an obstacle on left side, then move slight  right
      compareRPM(70, 50, v1, v2, 1, 1);
      delay(200);
      //compare_distances();
  }
  else if (C_distance < C_distance_Min && R_distance > S_distance_Min && L_distance > S_distance_Min){
    //If there is an obstacle on the front side, move back, compare left and right distances, select which distance to move into
      compareRPM(0, 0, v1, v2, 0, 0); //stop
      delay(200);
      compareRPM(50, 50, v1, v2, -1, -1); //move back
      delay(300);
      compareRPM(0, 0, v1, v2, 0, 0); //stop
    //call distance measuring functions  
      R_distance = UltraRead_R();
      L_distance = UltraRead_L();
      delay(100);
    //compare the distances
      if (L_distance > R_distance){
        //if distance on the left is greater than distance on the right, then move left
          compareRPM(50, 70, v1, v2, 1, 1);
          delay(200);
      }
      else if (R_distance > L_distance){
        //if distance on the right is greater than distance on the left, then move right
          compareRPM(70, 50, v1, v2, 1, 1); //move left
          delay(200);
          //compare_distances();
      }
      else{
        //rotate for to face back
          compareRPM(20, 20, v1, v2, -1, 1); 
          delay(200);
          compareRPM(0, 0, v1, v2, 0, 0); //stop
          delay(100);
          //compare_distances();
      }
  }

}









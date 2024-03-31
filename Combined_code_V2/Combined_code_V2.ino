#include<Servo.h>
#include<util/atomic.h>

//Encoder outputs for motor A connections
#define MotorA_ENCA 18
#define MotorA_ENCB 19
//Encoder outputs for motor B connections
#define MotorB_ENCA 2
#define MotorB_ENCB 3

//Defining Servo motor and ultrasonic sensor connected pins
#define TrigPin 33
#define EchoPin 35
#define ServoPin 10

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


// Defining variables to store the current distance
float curr_Distance = 0;



//Defining the constant target speed of every motor
float targetSpeed = 100;



void setup() {
  Serial.begin(9600);

  //Labels for the serial plotter motor encoder value outputs
  Serial.println("Motor_A_Speed, Motor_B_Speed");

  //Labels for the serial plotter motor encoder value outputs
  //Serial.println("Motor_A_Pos, Motor_B_Pos");

  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
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
  float v1 = velocity1/362.0*60.0;
  float v2 = velocity2/362.0*60.0;

  //LowPass Filter for the velocities at (25 Hz Cut off frequency)
  v1FiltA = 0.854*v1FiltA + 0.0728*v1 + 0.0728*v1PrevA;
  v1PrevA = v1;
  v2FiltB = 0.854*v2FiltB + 0.0728*v2 + 0.0728*v2PrevB;
  v2PrevB = v2;

  //a function called that will drive the two motors at 100rpm
  compareRPM(targetSpeed, v1, v2, 1, 1);

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

void compareRPM(int t_Speed, int rpm1, int rpm2, int dirA, int dirB){
  //compute the control signal u
  float kp1 = 12;
  float ki1 = 1;
  float kp2 = 8.5;
  float ki2 = 0.5;

  float error1 = t_Speed - rpm1;
  float error2 = t_Speed - rpm2;

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

/*int UltraRead(){
  //Read detected distance by ultrasonic
  //Triger the ultrasonic sensor low
  digitalWrite(TrigPin, LOW); 
  delayMicroseconds(2);

  //Triger the ultrasonic sensor High and delay for 10 micro-seconds then trig low
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  //Calculate distance from object
  long duration = pulseIn(EchoPin, HIGH);
  curr_Distance = duration * 0.0344 / 2; // Expression to calculate distance using time

  return curr_Distance;

}*/









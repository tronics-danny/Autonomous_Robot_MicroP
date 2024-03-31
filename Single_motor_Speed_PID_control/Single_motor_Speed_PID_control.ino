//Reading from the encoder
#include<util/atomic.h>

//Encoder outputs for motor A connections
#define MotorA_ENCA 18
#define MotorA_ENCB 19
//Encoder outputs for motor B connections
#define MotorB_ENCA 2
#define MotorB_ENCB 3


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
volatile int pos_i_A = 0;

volatile float velocity_i = 0;
volatile long prevT_i = 0;

//Defining global variables for the Raw and Filtered velocities
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

//gloabal variable to store the intergral error
float e_integral = 0;

void setup() {
  Serial.begin(9600);

  //Labels for the serial plotter motor encoder value outputs
  Serial.println("Motor_A_Pos, Motor_B_Pos");

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
  
  
  //Setting encoder output for motor A and B ENCA to trigger an interrupt then call a funtion
  attachInterrupt(digitalPinToInterrupt(MotorA_ENCA),readEncoderA,RISING);
  
}

void loop() {
  /*
  int pwm = 100/3.0*micros()/1.0e6;
  int dir = 1;

  setMotor(dir, pwm, enA, en1, en2);
  */

  //read the position in an atomic block to avoid potential misreads
  int pos = 0;
  float velocity2 = 0; // used for method 2

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i_A;
    velocity2 = velocity_i;

  }

  //compute velocity with method of counting pulses per time interval
  long currT = micros();
  float deltaT = ((float) (currT-prevT)) / 1.0e6;
  float velocity1 = (pos - posPrevA) / deltaT;
  posPrevA = pos;
  prevT = currT;

  //convert counts per sec to RPM
  float v1 = velocity1/362.0*60.0;
  float v2 = velocity2/362.0*60.0;

  //LowPass Filter for the velocities at (25 Hz Cut off frequency)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  //Set a target Speed
  float vt = 150 *(sin(currT/1e6)>0);

  //Compute the control signal u
  float kp = 5; // proportional term coefficient
  float ki = 100; // integral term coefiecient
  float e = vt - v1Filt;
  float e_integral = e_integral + e*deltaT;

  float u = kp*e + e_integral;

  //set motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwm = (int) fabs(u);
  //cut the pwm signal at 255
  if (pwm > 255){
    pwm = 255;
  }
  setMotor(dir, pwm, enA, en1, en2);

  Serial.print(v1);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);

}

void setMotor(int dir, int pwmVal, int m_pin, int in1, int in2){
  analogWrite(m_pin, pwmVal);

  if (dir==1){
    //Motor 2 to rotate in forward direction
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (dir==-1){
    //Motor to rotate in reverse direction 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else{
    //Motor to stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


//This function is called whenever ENCA triggers an interupt
//Allows to read the positon measurements from the encoder

void readEncoderA(){
  //Reading from the encoder output B
  int b = digitalRead(MotorA_ENCB);
  int increment = 0;
  //If function to icrement position value for forward direction and decrement position value for reverse rotation of the motor
  if (b>0){
    increment = 1;
  }
  else{
    increment = -1;
  }
  pos_i_A = pos_i_A + increment;

  //Compute velocity with method of measuring the time elapsed between triggers
  long currT = micros();
  float deltaT = ((float) (currT-prevT_i)) / 1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;

}

  

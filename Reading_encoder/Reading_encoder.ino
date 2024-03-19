//Reading from the encoder
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

//Creating a global variable to store the position of the motors A and B 
int motorA_pos = 0;
int motorB_pos = 0;

//Defining global storage variables that can be used to store values between time steps
long prevT = 0; //previous time
float eprev_MA = 0; //previous error for motor A
float eprev_MB = 0; //previous error for motor B
float eintegral_MA = 0; //integral error for motor A
float eintegral_MB = 0; //integral error for motor A

void setup() {
  Serial.begin(115200);

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
  attachInterrupt(digitalPinToInterrupt(MotorB_ENCA),readEncoderB,RISING);
  
}

void loop() {

  //Setting a target for the motor
  int target_MA = 1200;
  int target_MB = 1200;

  //Defining PID control constants
  float Kp = 1;
  float Kd = 0;
  float Ki = 0;

  // Computing the time difference
  //Current time in microseconds
  long currT = micros();
  //Calculating delta T
  float deltaT= ((float)(currT-prevT))/1.0e6;
  //setting the previous time to be the current time
  prevT = currT;

  //Calculating the error
  int e_MA = Motor_A_Pos-target_MA;
  int e_MB = Motor_B_Pos-target_MB;

  //Calcualting the derivative
  float derA = (e_MA-eprev_MA)/(deltaT);
  float derB = (e_MB-eprev_MB)/(deltaT);

  // Calculating the integral
  eintegral_MA = eintegral_MA + e_MA*deltaT;
  eintegral_MB = eintegral_MB + e_MB*deltaT;

  //Computing the control signal
  float u_MA = Kp*e_MA + Kd*derA + Ki*eintegral_MA;
  float u_MB = Kp*e_MB + Kd*derB + Ki*eintegral_MB;

  //calling the setMotor funtion to drive motor A
  setMotor(1, 100, enA, en1, en2);
  setMotor(-1, 100, enB, en3, en4);
  delay(1000);
  setMotor(-1, 100, enA, en1, en2);
  setMotor(+1, 100, enB, en3, en4);
  delay(1000);
  setMotor(0, 100, enA, en1, en2);
  setMotor(0, 100, enB, en3, en4);
  delay(1000);
  //Reading outputs from the encoder
  //Serial.print("Position for motor A: ");
  Serial.print(motorA_pos);
  Serial.print(",");
  //Serial.print("Position for motor B: ");
  Serial.print(motorB_pos);
  Serial.print(",");
  Serial.println();

}

// Function to set the motor direction and speed
// Parameters passed in the function are
  /*
  dir = set direction of rotation of the motor
  pwmVal = set value for the PWM signal to feed to the motor
  m_pin = Motor pin in which the PWM is set
  in1 = Input1 for the set motor
  in2 = Input2 for the set motor
  */
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
  //If function to icrement position value for forward direction and decrement position value for reverse rotation of the motor
  if (b>0){
    motorA_pos++;
  }
  else{
    motorA_pos--;
  }
}

void readEncoderB(){
  //Reading from the encoder output B
  int b = digitalRead(MotorB_ENCB);
  //If function to icrement position value for forward direction and decrement position value for reverse rotation of the motor
  if (b>0){
    motorB_pos++;
  }
  else{
    motorB_pos--;
  }
}

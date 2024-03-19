#include<Servo.h>

//Defining Servo motor and ultrasonic sensor connected pins
#define TrigPin 33
#define EchoPin 35
#define ServoPin 10

// Defining minimuma nd maximum angles of rotation of the servo motor
const int minAngle = 30;
const int maxAngle = 135;

// Defining speed of rotating the sedrvo motor
int step_Speed = 1;

// Defining variables to store the current distance and current angle
float curr_Distance = 0;
int curr_Angle = 0;

//Creating a class instance for the servo motor
Servo RadarMotor;

//Defining a variable to store min distance from the obstaclke that the robot can evade
const int minDistance = 15;

void setup() {
  Serial.begin(9600);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  pinMode(ServoPin, OUTPUT);
  RadarMotor.attach(ServoPin);

}

void loop() {
  MotorControl();

}

void MotorControl(){
  //Start by moving servo motor to minimum angle
  RadarMotor.write(minAngle);

  //Move motor to max angle in steps
  for (curr_Angle = minAngle; curr_Angle <= maxAngle; curr_Angle += step_Speed) { // goes from 30 degrees to 135 degrees
    // in steps of 1 degree
    RadarMotor.write(curr_Angle);              // tell servo to go to position in variable 'curr_Angle'
    delay(10);                       // waits 15ms for the servo to reach the position
    
    //Calculate distance from object for every angle turned
    CalculateDistance();
  }
  for (curr_Angle = maxAngle; curr_Angle >= minAngle; curr_Angle -= step_Speed) { // goes from 135 degrees to 30 degrees
    // in steps of 1 degree
    RadarMotor.write(curr_Angle);              // tell servo to go to position in variable 'curr_Angle'
    delay(10); // waits 15ms for the servo to reach the position
    
    //Calculate distance from object for every angle turned
    CalculateDistance();
  }

}

void CalculateDistance(){
  //Triger the ultrasonic sensor
  digitalWrite(TrigPin, LOW); //Trigerint the triger pin low first and delaying for 10 microseconds
  delayMicroseconds(2);

  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  //Calculate distance from object
  long duration = pulseIn(EchoPin, HIGH);
  curr_Distance = duration * 0.0344 / 2; // Expression to calculate distance using time
 
  //Gettinmg Distance at specific servo angles
  if (curr_Angle==30 || curr_Angle==56 || curr_Angle==82 || curr_Angle==108 || curr_Angle==135){
    Serial.print("Distance: ");
    Serial.print(curr_Distance); 
    Serial.println(" cm");
    Serial.print("Current Angle: ");
    Serial.print(curr_Angle); 
    Serial.println(" Degrees");
  }

}

//Functon to compare the different distances from different angles and deciding on what direction to turn
void DirAlg(){
  int c_Angle = curr_Angle;
  switch(c_Angle)
  {
    case 30:

    case 56:

    case 82:

    case 108:

    case 135:

    default:
    
  }
}

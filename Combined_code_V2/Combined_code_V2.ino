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

//Defining a variable to store min distance from the obstacle that the robot can evade
const int minDis_Lim = 15;

// Defining an array to store ultrasonic sensor values from whiuch we determine the direction the robot should move
int distanceArray[4]; //leaving the array empty so we can fill it with sensor data
int angleArray[4];

// Defining variables to store max and min values of the distance array
int max_Dis, min_Dis, b=0, a=0, currVal1, currVal2;


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


void ServoControl(int &angle){
  //Move servo motor to the angle indicated
  RadarMotor.write(angle);
}

int UltraRead(){
  //Read detected distance by altrasonic
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

}

void DirControl(){
  //Determine the direction in which the robot should go into
  //Variable to store the current distance from the obstacle
  int dist = 0;

  //move servo to make ultrasonic face in forward direction
  RadarMotor.write(angle0);
  dist = UltraRead();
  if(dist > minDis_Lim && dist>0)
    {
      //Stop the robot, check for obstacles in the other angles
      Stop();
      CompareDist();    
    }  
  else if(dist < minDis_Lim && dist>0)
    {
      //Stop the robot, check for obstacles in the other angles
      Stop();
      CompareDist();    
    }
  else
    {


    }

}

void CompareDist(){
    int angle1 = 30;
    int angle2 = 135;
    int angle0 = 82;

    //Move servo motor to the right
    RadarMotor.write(angle1);
    delay(500);
    int distR = UltraRead();
    delay(100);
    //Move servo motor to the left
    RadarMotor.write(angle2);
    delay(500);
    int distL = UltraRead();
    delay(100);
    //Move motor to face ultrasonic forward
    RadarMotor.write(angle0);
    
    if (distR > distL)
      {
        //Go back and turn right
        Go_Straight_Backward();
        delay(500);
        turn_Slight_Right();
        delay(350);
      }
    else if (distL > distR)
      {
        //Go back and turn left
        Go_Straight_Backward();
        delay(500);
        turn_Slight_Left();
        delay(350);
      }
    else
      {
        //Go back and turn left
        Go_Straight_Backward();
        delay(500);
        turn_Slight_Left();
        delay(350);
      } 

}   


void Stop(){
  //Function for stopping the robot
  Serial.println("STOP");

}
void Slow(){
  //Function for slowing the robot down
  Serial.println("SLOW DOWN");

}
void Go_Straight_Forward(){
  //Function for driving the robot straight forward
  Serial.println("STRAIGHT FORWARD");

}
void Go_Straight_Backward(){
  //Function for driving the robot straight backward
  Serial.println("STRAIGHT BACKWARD");
  
}
void turn_Slight_Left(){
  //Function for driving the robot slight left
  Serial.println("SLIGHT LEFT");
  
}
void turn_Slight_Right(){
  //Function for driving the robot slight right
  Serial.println("SLIGHT RIGHT");
  
}
void turn_Back_Right(){
  //Function for driving the robot back right
  Serial.println("BACK RIGHT");
  
}
void turn_Back_Left(){
  //Function for driving the robot back left
  Serial.println("BACK LEFT");
  
}





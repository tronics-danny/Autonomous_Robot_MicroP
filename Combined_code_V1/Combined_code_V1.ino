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
  //Declaring variables to store the different angles
  int angle1 = 30;
  int angle2 = 56;
  int angle3 = 82;
  int angle4 = 108;
  int angle5 = 135;

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
  if (curr_Angle==angle1 || curr_Angle==angle2 || curr_Angle==angle3 || curr_Angle==angle4 || curr_Angle==angle5)
  {
        //satoring the distances read from the ultrasonic sensor to an array
        if(curr_Angle==angle1)
          {
            distanceArray[0] = curr_Distance;
            angleArray[0] = angle1;
          } 
        else if(curr_Angle==angle2)
          {
            distanceArray[1] = curr_Distance;
            angleArray[1] = angle2;
          }
        else if(curr_Angle==angle3)
          {
            distanceArray[2] = curr_Distance;
            angleArray[2] = angle3;
          }
        else if(curr_Angle==angle4)
          {
            distanceArray[3] = curr_Distance;
            angleArray[3] = angle4;
          }
        else if(curr_Angle==angle5)
          {
            distanceArray[4] = curr_Distance;
            angleArray[4] = angle5;
          }
        else
          {
            //do nothing      
          }
      //Accessing the distances in the arrays and printing them on the serial monitor
      //Determining the max distance recorder and the min distance recorded
      for(int i=0; i<5; i++)
        {
          Serial.print("Distance: ");
          Serial.print(distanceArray[i]);
          Serial.println(" cm");
          Serial.print("Current Angle: ");
          Serial.print(angleArray[i]); 
          Serial.println(" Degrees");
          //Returning the max distance recorded
          currVal1 = max(distanceArray[i],b);
          b = currVal1;          
        }
        max_Dis = b;
        a = max_Dis;

        for(int i=0; i<5; i++){
          currVal2 = min(distanceArray[i],a);
          a = currVal2;
        }
        min_Dis = a;

        // Dont forget to call direction determinant function of the robot here  

  }

}

//Functon to compare the different distances from different angles and deciding on what direction to turn
void DirAlg(){

}

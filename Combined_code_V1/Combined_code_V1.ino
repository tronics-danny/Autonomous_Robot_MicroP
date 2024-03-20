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
  int angle1 = 30;
  int angle2 = 56;
  int angle3 = 82;
  int angle4 = 108;
  int angle5 = 135;

  //Variable to store the current distance from the obstacle
  int dist = 0;

  ServoControl(angle3);
  dist = UltraRead();
  if(dist<minDis_Lim)
    {
      //Stop the robot, check for obstacles in the other angles
      Stop();
      ServoControl(angle2);
      dist = UltraRead();
      

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
  /*int angle1 = 30;
  int angle2 = 56;
  int angle3 = 82;
  int angle4 = 108;
  int angle5 = 135;*/

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
        //storing the distances read from the ultrasonic sensor to an array
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
      det_Direction();  

  }

}

//Functon to compare the different distances from different angles and deciding on what direction to turn
void det_Direction(){
  max_Dis = max_Dis;
  min_Dis = min_Dis;

      if (min_Dis > minDis_Lim && max_Dis > minDis_Lim)
        {
          //Call funtion to move the robot strait forward
          Serial.println("Call funtion to move the robot strait forward");

        }
        else if((distanceArray[0] <= minDis_Lim) && (distanceArray[1] > minDis_Lim && distanceArray[2] > minDis_Lim))
        {
          // Call a function to turn the robot slight left until the distance from the obstacle is large enough for robot to pass
          Serial.println("Call a function to turn the robot slight left");

        }
        else if((distanceArray[4] <= minDis_Lim) && (distanceArray[3] > minDis_Lim && distanceArray[2] > minDis_Lim))
        {
          // Call a function to turn the robot slight right until the distance from the obstacle is large enough for robot to pass
          Serial.println("Call a function to turn the robot slight right");

        }
        else if((distanceArray[2] <= 18) && (distanceArray[3] > 18 && distanceArray[4] > minDis_Lim))
        {
          // Call a function to turn the robot slight left 
          Serial.println("Call a function to turn the robot slight left");
        }
      else if((distanceArray[2] <= 18) && (distanceArray[1] > 18 && distanceArray[0] > minDis_Lim))
        {
          // Call a function to turn the robot slight right
          Serial.println("Call a function to turn the robot slight right"); 

        }
        else if (max_Dis <= minDis_Lim)
        {
          // Call a function to reverse the robot straight
          Serial.println("Call a function to reverse the robot straight");

        }
        else
        {
          // Go forward
          Serial.println("Go forward");

        }

}



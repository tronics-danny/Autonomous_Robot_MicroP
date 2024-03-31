//Obstacle avoiding robot with 3 ultra-sonic sensors
#include <Servo.h>
Servo servo;
#define trigPinL 33  //pin defination of the left ultra-sonic sensor
#define echoPinL 35 
#define trigPinC 37  //pin defination of the center ultra-sonic sensor
#define echoPinC 39 
#define trigPinR 41   //pin defination of the right ultra-sonic sensor
#define echoPinR 43 
#define servo 10
#define enA 4//Enable1 L298N Pin enA 
#define in1 5 //Motor1  L298 Pin in1 
#define in2 6 //Motor1  L298 Pin in1 
#define in3 7 //Motor2  L298 Pin in1 
#define in4 8 //Motor2  L298 Pin in1 
#define enB 9 //Enable2 L298N Pin enB 

long duration, distance;

//code that runs only once
void setup()
{
  Serial.begin(9600);
  pinMode(in1, OUTPUT);     // Set Motor Pins As O/P
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(trigPinL, OUTPUT); // Set trigPinL As O/P To Transmit Waves
  pinMode(echoPinL, INPUT);  //Set echoPinL As I/P To Receive Reflected Waves
  pinMode(trigPinC, OUTPUT); // Set trigPinC As O/P To Transmit Waves
  pinMode(echoPinC, INPUT);  //Set echoPinC As I/P To Receive Reflected Waves
  pinMode(trigPinR, OUTPUT); // Set trigPinR As O/P To Transmit Waves
  pinMode(echoPinR, INPUT);  //Set echoPinR As I/P To Receive Reflected Waves
  //servo.attach(10);
}

//code that runs continously
void loop()
{
  int leftSensor = sensorOne();
  int centerSensor = sensorTwo();
  int rightSensor = sensorThree();

  //check distance
  if(centerSensor <= 15)
  {
    stop();
    Serial.println("Stop");
    delay(1000);
    if(leftSensor > rightSensor)
    {
      turnLeft();
      Serial.print("Left");
      delay(500);
    }
    else
    {
      turnRight();
      Serial.print("Right");
      delay(500);
    }
  }
  Serial.println("Forward");
  forward();
}

//Get the value from left sensor
int sensorOne()
{
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinL, HIGH); //transmit wave for 10us
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);

  duration = pulseIn(echoPinL, HIGH);
  distance = duration*0.034/2; //convert time to distance
  return distance; //return the value from the sensor
}

//Get the value from center sensor
int sensorTwo()
{
  digitalWrite(trigPinC, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinC, HIGH); //transmit wave for 10us
  delayMicroseconds(10);
  digitalWrite(trigPinC, LOW);

  duration = pulseIn(echoPinC, HIGH);
  distance = duration*0.034/2; //convert time to distance
  return distance; //return the value from the sensor
}

//Get the value from right sensor
int sensorThree()
{
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinR, HIGH); //transmit wave for 10us
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);

  duration = pulseIn(echoPinR, HIGH);
  distance = duration*0.034/2; //convert time to distance
  return distance; //return the value from the sensor
}

void forward()
{
    //servoPulse(servo, 82);
    analogWrite(enA,110);
    analogWrite(enB,80);
    digitalWrite(in1, HIGH); //Left Motor backword Pin 
    digitalWrite(in2, LOW); //Left Motor forword Pin 
    digitalWrite(in3, LOW); //Right Motor forword Pin 
    digitalWrite(in4, HIGH); //Right Motor backword Pin  
}

void stop()
{
    digitalWrite(in1, LOW); //Left Motor backword Pin 
    digitalWrite(in2, LOW); //Left Motor forword Pin 
    digitalWrite(in3, LOW); //Right Motor forword Pin 
    digitalWrite(in4, LOW); //Right Motor backword Pin 
    //delay(100);
}

void turnRight()
{ 
  analogWrite(enA,150);
  analogWrite(enB,40);
  digitalWrite(in1, HIGH); //Left Motor backword Pin 
  digitalWrite(in2, LOW); //Left Motor forword Pin 
  digitalWrite(in3, LOW); //Right Motor forword Pin 
  digitalWrite(in4, HIGH); //Right Motor backword Pin 
}

void turnLeft()
{ 
  analogWrite(enA,60);
  analogWrite(enB,150);
  digitalWrite(in1, HIGH); //Left Motor backword Pin 
  digitalWrite(in2, LOW); //Left Motor forword Pin 
  digitalWrite(in3, LOW); //Right Motor forword Pin 
  digitalWrite(in4, HIGH); //Right Motor backword Pin 
}

void backward()
{
  analogWrite(enA,110);
  analogWrite(enB,80);
  digitalWrite(in1, LOW); //Left Motor backword Pin 
  digitalWrite(in2, HIGH); //Left Motor forword Pin 
  digitalWrite(in3, HIGH); //Right Motor forword Pin 
  digitalWrite(in4, LOW); //Right Motor backword Pin 
  delay(500);
}
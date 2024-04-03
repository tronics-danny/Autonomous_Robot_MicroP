//Obstacle avoiding robot with 3 ultra-sonic sensors
#include <Servo.h>
Servo servo;
#define trigPinL 38  //pin defination of the left ultra-sonic sensor
#define echoPinL 39 
#define trigPinC 33  //pin defination of the center ultra-sonic sensor
#define echoPinC 35 
#define trigPinR 36   //pin defination of the right ultra-sonic sensor
#define echoPinR 37 
#define servo 10
#define enA 4//Enable1 L298N Pin enA 
#define in1 5 //Motor1  L298 Pin in1 
#define in2 6 //Motor1  L298 Pin in1 
#define in3 7 //Motor2  L298 Pin in1 
#define in4 8 //Motor2  L298 Pin in1 
#define enB 9 //Enable2 L298N Pin enB 
int green1 = 32;
int green2 = 28;
int red = 30;

long duration; 
float distance;
int dis_min = 21;
int dis_min_side = 25;

//code that runs only once
void setup()
{
  Serial.begin(9600);
  pinMode(in1, OUTPUT);     // Set Motor Pins As O/P
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(green1, OUTPUT);
  pinMode(green2, OUTPUT);
  pinMode(red, OUTPUT);
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
  //check distance{
  int C_distance = sensorTwo();
  int R_distance = sensorOne();
  int L_distance = sensorThree();
if(C_distance>dis_min)
{
  if(R_distance<dis_min_side && L_distance<dis_min_side){
    forward();
  }
  else if(R_distance>dis_min_side && L_distance>dis_min_side){
    forward();
  }
  else if(R_distance<dis_min_side && L_distance>dis_min_side){
    turnLeft();
    delay(100);
  }
  else if(R_distance>dis_min_side && L_distance<dis_min_side){
    turnRight();
    delay(100);  
  }
  else{
    backward();
    delay(700);
    stop();
    delay(300);
    loop();
  }

}
else if(C_distance<dis_min)
{
  if(R_distance<dis_min_side && L_distance<dis_min_side){
    if(R_distance < L_distance){
      turnLeft();
      delay(200);
    }
    else if(R_distance > L_distance){
      turnRight();
      delay(200);
    }
    else{
    stop();
    delay(400);
    backward();
    delay(700);
    stop();
    delay(300);
    loop(); 
    }  

  }
  else if(R_distance < dis_min_side && L_distance > dis_min_side){
    turnLeft();
    delay(200);
  }
  else if(R_distance>dis_min_side && L_distance<dis_min_side){
    turnRight();
    delay(200);
  }
  else{
    backward();
    delay(700);
    stop();
    delay(300);
    turnRight();
  }
}
else{
  loop();
}
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
  distance = duration*0.0344/2; //convert time to distance
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
  distance = duration*0.0344/2; //convert time to distance
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
  distance = duration*0.0344/2; //convert time to distance
  return distance; //return the value from the sensor
}

void forward()
{
    //servoPulse(servo, 82);
    analogWrite(enA,130);
    analogWrite(enB,90);
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
    stopLED(); 
    //delay(1000);
}

void turnRight()
{ 
  analogWrite(enA,150);
  analogWrite(enB,30);
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

void forwardLED()
{
  digitalWrite(green1, LOW);
  digitalWrite(green2, LOW);
  delay(500);
  digitalWrite(green1, HIGH);
  digitalWrite(green2, HIGH);
  delay(200);
}

void stopLED()
{
  digitalWrite(red, LOW);
  delay(1000);
  digitalWrite(red, HIGH);
  delay(200);
}
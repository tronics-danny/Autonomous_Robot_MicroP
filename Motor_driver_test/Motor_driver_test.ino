//motor A
int enA = 4;
int en1 = 5;
int en2 = 6;

//motor B
int enB = 9;
int en3 = 8;
int en4 = 7;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(en3, OUTPUT);
  pinMode(en4, OUTPUT);

}

void loop() {
  delay(1000);
  
  Go_Straight_Forward();
  delay(3000);
  Stop();
  delay(5000);

  //Go_Straight_Backward();
  //turn_Slight_Left();
  //turn_Slight_Right();
  //turn_Back_Left();
  //turn_Back_Right();

}
void Go_Straight_Forward(){
  //Function for driving the robot straight forward
  Serial.println("STRAIGHT FORWARD");

  digitalWrite(en1, HIGH);
  digitalWrite(en2, LOW);
  digitalWrite(en3, HIGH);
  digitalWrite(en4, LOW);

  analogWrite(enA, 120);
  analogWrite(enB, 85);

}

void Stop(){
  //Function for stopping the robot
  Serial.println("STOP");

  digitalWrite(en1, LOW);
  digitalWrite(en2, LOW);
  digitalWrite(en3, LOW);
  digitalWrite(en4, LOW);

}
void Slow(){
  //Function for slowing the robot down
  Serial.println("SLOW DOWN");
  
  digitalWrite(en1, HIGH);
  digitalWrite(en2, LOW);
  digitalWrite(en3, HIGH);
  digitalWrite(en4, LOW);

  analogWrite(enA, 25);
  analogWrite(enB, 25);

}

void Go_Straight_Backward(){
  //Function for driving the robot straight backward
  Serial.println("STRAIGHT BACKWARD");

  digitalWrite(en1, HIGH);
  digitalWrite(en2, LOW);
  digitalWrite(en3, HIGH);
  digitalWrite(en4, LOW);

  analogWrite(enA, 25);
  analogWrite(enB, 25);
  
}
void turn_Slight_Left(){
  //Function for driving the robot slight left
  Serial.println("SLIGHT LEFT");
  
  digitalWrite(en1, HIGH);
  digitalWrite(en2, LOW);
  digitalWrite(en3, HIGH);
  digitalWrite(en4, LOW);

  analogWrite(enA, 25);
  analogWrite(enB, 25);
  
}
void turn_Slight_Right(){
  //Function for driving the robot slight right
  Serial.println("SLIGHT RIGHT");
  
  digitalWrite(en1, HIGH);
  digitalWrite(en2, LOW);
  digitalWrite(en3, HIGH);
  digitalWrite(en4, LOW);

  analogWrite(enA, 25);
  analogWrite(enB, 25);
  
}
void turn_Back_Right(){
  //Function for driving the robot back right
  Serial.println("BACK RIGHT");

  digitalWrite(en1, HIGH);
  digitalWrite(en2, LOW);
  digitalWrite(en3, HIGH);
  digitalWrite(en4, LOW);

  analogWrite(enA, 25);
  analogWrite(enB, 25);
  
}
void turn_Back_Left(){
  //Function for driving the robot back left
  Serial.println("BACK LEFT");

  digitalWrite(en1, HIGH);
  digitalWrite(en2, LOW);
  digitalWrite(en3, HIGH);
  digitalWrite(en4, LOW);

  analogWrite(enA, 25);
  analogWrite(enB, 25);
  
}

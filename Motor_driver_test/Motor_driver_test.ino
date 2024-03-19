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
  //Turn on motor A
  digitalWrite(en1, HIGH);
  digitalWrite(en2, LOW);
  digitalWrite(en3, HIGH);
  digitalWrite(en4, LOW);

  analogWrite(enA, 25);
  analogWrite(enB, 25);

}

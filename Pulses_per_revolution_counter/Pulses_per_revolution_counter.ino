/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: Count the number of encoder pulses per revolution.  
 */
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
//Encoder outputs for motor A connections
#define MotorA_ENCA 2
#define MotorA_ENCB 3
 
// Keep track of the number of right wheel pulses
volatile long motor_A_pulse_count = 0;
 
void setup() {
 
  // Open the serial port at 9600 bps
  Serial.begin(9600); 
 
  // Set pin states of the encoder
  pinMode(MotorA_ENCA, INPUT_PULLUP);
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(MotorA_ENCA), right_wheel_pulse, RISING);
   
}
 
void loop() {
  
    Serial.print(" Pulses: ");
    Serial.println(motor_A_pulse_count);  
}
 
// Increment the number of pulses by 1
void right_wheel_pulse() {
  motor_A_pulse_count++;
}
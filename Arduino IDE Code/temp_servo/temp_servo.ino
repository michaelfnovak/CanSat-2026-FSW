#include <Servo.h>

Servo myservo;  // create servo object to control a servo
const int servoPin = 9;
const int tmp36pin = A1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo.attach(servoPin);
  pinMode(tmp36pin, INPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int adcValue = analogRead(tmp36pin);
  
  float voltage = (5.0/1024.0)*adcValue;      //to be edited by the student!
  float temperature = (voltage - 0.5)/(0.01);  //to be edited by the student!

  int angle = map(temperature, 20, 30, 0, 180); // scale it to use it with the servo (value between 0 and 180)
  myservo.write(angle);                       // sets the servo position according to the scaled value
  delay(200);
  Serial.print(adcValue);
  Serial.print(" ");
  Serial.print(temperature);
  Serial.print(" "); 
  Serial.print(angle);
  Serial.println();
  delay(500); 
}

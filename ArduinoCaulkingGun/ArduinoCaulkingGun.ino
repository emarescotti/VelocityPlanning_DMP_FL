/*
    Master Thesis - Caulking Gun Code
    Marescotti Elia  -  29/09/2020

    SPDX-License-Identifier: AGPL-3.0-or-later
*/

#include <Servo.h>
Servo motor;

const int pinServo = 4;
const int pinPot = A1;
const int pinLed = 3;   // just for debug

int pot = 512;    // potentiometer reading
String text = ""; // string value from the serial port
int alpha = 0;    // gun trigger pression [0-99]
int muS = 1300;   // microseconds as input for the motor

bool MODE = 1;    // CHOOSE: 0 = potentiometer
                  //         1 = serial input
void setup(){
  Serial.begin(115200);
  motor.attach(pinServo);
  
  // JUST TO SEE EVERYTHING IS WORKING
  digitalWrite(pinLed,HIGH);
  delay(100);
  digitalWrite(pinLed,LOW);
  delay(100);
  digitalWrite(pinLed,HIGH);
  delay(100);
  digitalWrite(pinLed,LOW);
  
  motor.writeMicroseconds(muS);
  }

void loop(){
  if(MODE==0) { // POTENTIOMETER
    pot = analogRead(pinPot);
    muS = map(pot,20,1000,1300,1930); // 1400-1930
    motor.writeMicroseconds(muS);
    delay(10);
    } else { // SERIAL READING
      while (Serial.available() > 0) {
      char inChar = Serial.read();
      text += inChar;

      // if you get a newline
      if (inChar == '\n') {
        alpha = text.toInt();
  
        // MOTOR COMMAND
        muS = map(alpha,0,99,1300,1900); // 1400-1930
        motor.writeMicroseconds(muS);
      
        // clear the string for new input:
        text = "";
        }
      }
    }  
  }

  

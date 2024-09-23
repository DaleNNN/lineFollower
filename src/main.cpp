#include <Arduino.h>
#include <QTRSensors.h>

//Sensor declaration left, right, midleft, midright
constexpr int sensorL = A2;
constexpr int sensorR = A3;
constexpr int sensorML = A0;
constexpr int sensorMR = A1;


constexpr int switchPin = 3; //Pin for on/off switch


constexpr int motorLEnable = 10; //Enabler and speed for left motor
constexpr int motorLPos = 8;
constexpr int motorLNeg = 12; //Motors left and right pos and neg-pins
constexpr int motorRPos = 13;
constexpr int motorRNeg = 9;
constexpr int motorREnable = 8; //Enabler and speed for right motor

//Speed to be used later
int leftSpeed = 0;
int rightSpeed = 0;

int threshold = 800; //Threshold for reflectance
int readingL = 0;
int readingR = 0; //Declaring readings variables for all sensors
int readingML = 0;
int readingMR = 0;


void setup() {
    pinMode(motorLEnable, OUTPUT);
    pinMode(motorLPos, OUTPUT);
    pinMode(motorLNeg, OUTPUT);
    pinMode(motorREnable, OUTPUT);
    pinMode(motorRPos, OUTPUT);
    pinMode(motorRNeg, OUTPUT);
}

//Declaration of functions to be implemented later
void stopMotors();
void motorsForward();
void serialLog();

void loop() {
    threshold = 800;
    Serial.begin(9600);

    //Sensor readings
    readingL = analogRead(sensorL);
    readingR = analogRead(sensorR);
    readingML = analogRead(sensorML);
    readingMR = analogRead(sensorMR);

    serialLog();

    if (digitalRead(switchPin) == HIGH) {
        //If on/off switch is set to on

        if ((readingL > threshold) && (readingR > threshold)
            && (readingML > threshold) && (readingMR > threshold)) {
            //If no line is detected
            stopMotors();

        } else if ((readingL > threshold) && (readingML > threshold)
                   && (readingR < threshold)) {
            //If right detects line
            leftSpeed = 255;
            rightSpeed = 100;

        } else if ((readingL < threshold) && (readingMR > threshold)
                   && (readingR > threshold)) {
            //If left detects line
            leftSpeed = 100;
            rightSpeed = 255;

        } else if ((readingL > threshold) && (readingML < threshold)
                   && (readingMR > threshold) && (readingR > threshold)) {
            //If only midleft detects line
            leftSpeed = 175;
            rightSpeed = 255;

        } else if ((readingL > threshold) && (readingML > threshold)
                   && (readingMR < threshold) && (readingR > threshold)) {
            //If only midright detects line
            leftSpeed = 255;
            rightSpeed = 175;

        } else {
            //If none of the above, go forward
            leftSpeed = 255;
            rightSpeed = 255;
        }
        motorsForward();
        delay(50);

    } else {
        //If on/off switch is set to off position
        stopMotors();
    }
}

void serialLog() {
    //Log the line position in terminal based off sensor readings
    if (readingL > threshold) {
        Serial.print("|  |");
    } else {
        Serial.print("|II|");
    }
    if (readingML > threshold) {
        Serial.print("|  |");
    } else {
        Serial.print("|II|");
    }
    if (readingMR > threshold) {
        Serial.print("|  |");
    } else {
        Serial.print("|II|");
    }
    if (readingR > threshold) {
        Serial.print("|  |");
    } else {
        Serial.print("|II|");
    }

    Serial.println();

    if (readingML > threshold && readingMR > threshold) {
        Serial.println(readingML);
    } else {
        Serial.println(readingML);
        Serial.println(readingMR);
        Serial.println(readingL);
        Serial.println(readingR);
    }

    delay(100);
}


void motorsForward() {
    //The function actually driving the motors based on speed previously set

    //Serial.println("Driving Motor forward with speed:" + String(leftSpeed) + " and  " + String(rightSpeed));

    digitalWrite(motorLPos, HIGH);
    digitalWrite(motorLNeg, LOW);
    digitalWrite(motorRPos, HIGH);
    digitalWrite(motorRNeg, LOW);

    // Set motor speed (0 to 255)
    analogWrite(motorLEnable, leftSpeed);
    analogWrite(motorREnable, rightSpeed);
}

void stopMotors() {
    analogWrite(motorLEnable, 0);
    analogWrite(motorREnable, 0);
}

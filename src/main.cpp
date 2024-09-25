#include <Arduino.h>
#include <QTRSensors.h>

//Creating object qtr
QTRSensors qtr;
constexpr uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];


constexpr int motorLEnable = 10; //Enabler and speed for left motor
constexpr int motorLPos = 8;
constexpr int motorLNeg = 12; //Motors left and right pos and neg-pins
constexpr int motorRPos = 13;
constexpr int motorRNeg = 9;
constexpr int motorREnable = 11; //Enabler and speed for right motor

//Speed to be used later
int leftSpeed = 0;
int rightSpeed = 0;

int threshold = 800; //Threshold for reflectance
int readingL = 0;
int readingR = 0; //Declaring reading variables for all sensors
int readingML = 0;
int readingMR = 0;


void setup() {
    // configuring the sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3}, SensorCount);
    qtr.setEmitterPin(3);

    Serial.begin(9600);

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
    //read sensor readings
    qtr.read(sensorValues);
    threshold = 800;

    readingL = sensorValues[2];
    readingR = sensorValues[3];
    readingML = sensorValues[0];
    readingMR = sensorValues[1];


    // *FOR DEBUG*
    // print the sensor values as numbers from 0 to 2500, where 0 means maximum
    // reflectance and 2500 means minimum reflectance
    /*for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();*/


    serialLog();


    if ((readingL > threshold) && (readingR > threshold)
        && (readingML > threshold) && (readingMR > threshold)) {
        //If no line is detected
        stopMotors();
    } else if ((readingL > threshold) && (readingML > threshold)
               && (readingMR < threshold) && (readingR < threshold)) {
        //If right and midright detects line
        leftSpeed = 255;
        rightSpeed = 100;
    } else if ((readingL < threshold) && (readingML < threshold)
               && (readingMR > threshold) && (readingR > threshold)) {
        //If left and midleft detects line
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
    } else if ((readingL < threshold) && (readingML > threshold)) {
        //If only left detects line
        leftSpeed = 50;
        rightSpeed = 255;
    } else if ((readingR < threshold) && readingMR > threshold) {
        //If only right detects line
        leftSpeed = 255;
        rightSpeed = 50;
    } else {
        //If none of the above, go forward
        leftSpeed = 255;
        rightSpeed = 255;
    }
    motorsForward();
}


void serialLog() {
    //Log the line position in terminal based off sensor readings
    if (readingL < threshold) {
        Serial.print("|  |");
    } else {
        Serial.print("|II|");
    }
    if (readingML < threshold) {
        Serial.print("|  |");
    } else {
        Serial.print("|II|");
    }
    if (readingMR < threshold) {
        Serial.print("|  |");
    } else {
        Serial.print("|II|");
    }
    if (readingR < threshold) {
        Serial.print("|  |");
    } else {
        Serial.print("|II|");
    }

    Serial.println();

    /*if (readingML > threshold && readingMR > threshold) {
        Serial.println(readingML);
    } else {
        Serial.println(readingML);
        Serial.println(readingMR);
        Serial.println(readingL);
        Serial.println(readingR);
    }*/

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

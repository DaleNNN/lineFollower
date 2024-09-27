#include <Arduino.h>
#include <QTRSensors.h>

//Creating object sensors
QTRSensors sensors;
constexpr uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];


constexpr int motorLEnable = 10; //Enabler and speed(PWM) for left motor
constexpr int motorLPos = 8;
constexpr int motorLNeg = 12; //Motors left and right pos and neg-pins
constexpr int motorRPos = 13;
constexpr int motorRNeg = 9;
constexpr int motorREnable = 11; //Enabler and speed(PWM) for right motor

//Speed to be used later
int leftSpeed = 0;
int rightSpeed = 0;

int threshold = 1000; //Threshold for reflectance
int readingL = 0;
int readingR = 0; //Declaring reading variables for all sensors
int readingML = 0;
int readingMR = 0;


void setup() {
    // configuring the sensors
    sensors.setTypeRC();
    sensors.setSensorPins((const uint8_t[]){A0, A1, A2, A3}, SensorCount);
    sensors.setEmitterPin(3);

    Serial.begin(9600);

    pinMode(motorLEnable, OUTPUT);
    pinMode(motorLPos, OUTPUT);
    pinMode(motorLNeg, OUTPUT);
    pinMode(motorREnable, OUTPUT);
    pinMode(motorRPos, OUTPUT);
    pinMode(motorRNeg, OUTPUT);

    delay(500);
}

//Declaration of functions to be implemented later
void stopMotors();
void motorsForward();
void serialLog();

void loop() {
    //read sensor readings
    sensors.read(sensorValues);
    threshold = 1000;


    //Save each reading in own variable
    readingL = sensorValues[3];
    readingR = sensorValues[2];
    readingML = sensorValues[1];
    readingMR = sensorValues[0];


    // *FOR DEBUG*
    // print the sensor values as numbers from 0 to 2500, where 0 means maximum
    // reflectance and 2500 means minimum reflectance
    for (uint8_t i = 3; i < SensorCount; i++)
    {
        Serial.print(String(readingL) + " " + String(readingML) + " " + String(readingMR) + " " + String(readingR));
        Serial.print('\t');
    }
    Serial.println();


    //serialLog();


    if ((readingL < threshold) && (readingR < threshold)
        && (readingML < threshold) && (readingMR < threshold)) {
        //If no line is detected
        stopMotors();
    } else if ((readingL < threshold) && (readingML < threshold)
               && (readingMR < threshold) && (readingR > threshold)) {
        //If right and midright detects line
        leftSpeed = 255;
        rightSpeed = 80;
    } else if ((readingL > threshold) && (readingML > threshold)
               && (readingMR < threshold) && (readingR < threshold)) {
        //If left and midleft detects line
        leftSpeed = 80;
        rightSpeed = 255;
    } else if ((readingL < threshold) && (readingML > threshold)
               && (readingMR < threshold) && (readingR < threshold)) {
        //If only midleft detects line
        leftSpeed = 160;
        rightSpeed = 255;
    } else if ((readingL < threshold) && (readingML < threshold)
               && (readingMR > threshold) && (readingR < threshold)) {
        //If only midright detects line
        leftSpeed = 255;
        rightSpeed = 160;
    } else if ((readingL > threshold) && (readingML < threshold)
            && (readingMR < threshold) && (readingR < threshold)) {
        //If only left detects line
        leftSpeed = 0;
        rightSpeed = 255;
    } else if ((readingR > threshold) && (readingL < threshold)
            && (readingMR < threshold) && (readingML < threshold)) {
        //If only right detects line
        leftSpeed = 255;
        rightSpeed = 0;
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

    delay(100);  //Necessary to not destroy console with spam of insanity.
                    //Leave serialLog function only for debugs
}


void motorsForward() {
    //Driving the motors with previously determined speed

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

#include <Arduino.h>
#include <Servo.h>

// Pin definitions
const int servoPinAzimuth = 0;
const int servoPinElevation = 1;
const int ledPinGreen = 2;
const int ledPinRed = 3;
const int resetPin = 15;
// LEDs are active LOW (RGB with common anode)

// Servo objects
Servo servoAzimuth;
Servo servoElevation;

// Servo positions
int azimuthPosition = 90; // Starting at center
int elevationPosition = 90; // Starting at center

// Constants
const int CENTER_TOLERANCE = 90; // Tolerance for being "centered"
const int BIGSTEP_THRESHOLD = 165; // Offsets greater than this cause bigger movement to keep tracking
const int STEP_SIZE = 1;         // Step size for servo adjustments
const int MIN_POSITION = 0;
const int MAX_POSITION = 180;

void setup() {
    // Initialize reset
    pinMode(resetPin, INPUT_PULLDOWN);

    // Initialize servos
    servoAzimuth.attach(servoPinAzimuth);
    servoElevation.attach(servoPinElevation);

    // Initialize LEDs
    pinMode(ledPinGreen, OUTPUT);
    pinMode(ledPinRed, OUTPUT);

    // Move servos to initial positions
    servoAzimuth.write(azimuthPosition);
    servoElevation.write(elevationPosition);

    // Initialize serial communication
    Serial.begin(9600);
}

void loop() {
    sleep_ms(10);
    if(digitalRead(resetPin)){
        servoAzimuth.write(90);
        servoElevation.write(90);
    }
  //while(true){
  //  servoAzimuth.write(80);
  //  sleep_ms(300);
  //  servoAzimuth.write(100);
  //  servoElevation.write(0);
  //  sleep_ms(100);
  //  servoElevation.write(180);
  //  
  //}
    if (Serial.available() > 0) {
        // Read the incoming data
        String data = Serial.readStringUntil('\n');
        data.trim(); // Remove any trailing whitespace

        // Parse the data
        // Find position of delimiter
        int separatorIndex = data.indexOf(',');
        if (separatorIndex != -1) {
            String offsetXStr = data.substring(0, separatorIndex);
            String offsetYStr = data.substring(separatorIndex + 1);

            int offsetX = offsetXStr.toInt();
            int offsetY = offsetYStr.toInt();
        

            // Adjust servos based on offsets
            if (abs(offsetX) > CENTER_TOLERANCE) {
                if(offsetX > 0){
                    digitalWrite(ledPinGreen, HIGH);
                    digitalWrite(ledPinRed, LOW);
                    if(offsetX > BIGSTEP_THRESHOLD){
                        azimuthPosition = azimuthPosition - STEP_SIZE * 2;
                    }else{
                        azimuthPosition = azimuthPosition - STEP_SIZE;
                    }
                }
                else{
                    digitalWrite(ledPinGreen, LOW);
                    digitalWrite(ledPinRed, HIGH);
                    if(offsetX > BIGSTEP_THRESHOLD){
                        azimuthPosition = azimuthPosition + STEP_SIZE * 2;
                    }else{
                        azimuthPosition = azimuthPosition + STEP_SIZE;
                    }
                }
                servoAzimuth.write(azimuthPosition);
                sleep_ms(30);
            }

            //if (abs(offsetY) > CENTER_TOLERANCE) {
            //    elevationPosition = servoElevation.read();
            //    elevationPosition = elevationPosition + offsetY;
            //    servoElevation.write(elevationPosition);
            //}

            // Control LEDs
            if (abs(offsetX) <= CENTER_TOLERANCE && abs(offsetX) != 0) {
                digitalWrite(ledPinGreen, HIGH);
                digitalWrite(ledPinRed, HIGH);
            }
    }
}
}

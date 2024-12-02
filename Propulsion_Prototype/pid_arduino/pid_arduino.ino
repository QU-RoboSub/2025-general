#include <Servo.h>
#include "ArduPID.h"
#include "ping1d.h"
#include "SoftwareSerial.h"

const byte servoPins[] = {3, 5, 6, 10}; // signal pins for the four ESCs
Servo servos[4];

static const uint8_t arduinoRxPin = 11;
static const uint8_t arduinoTxPin = 9;
SoftwareSerial pingSerial = SoftwareSerial(arduinoRxPin, arduinoTxPin);
static Ping1D ping { pingSerial };

ArduPID myController;

const int MIN_DEPTH = 400; // in mm, how close the submarine can be to the bottom of the pool
const int MAX_DEPTH = 600; // in mm, how far the auv can be from the bottom of the pool, set this to avoid jumping out the pool

const float CONFIDENCE_THRESH = 0; // Remove for later
const float FACTOR = 4.55; // Remove for later

double input;
double output;

// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 500;
double p = 33.43;
double i = 4;
double d = 42.01;

void setup() {
  pingSerial.begin(9600);
  Serial.begin(115200); // Faster serial communication for real-time debugging

  Serial.println("Starting PID Controller initialization...");
  myController.begin(&input, &output, &setpoint, p, i, d);
  myController.setOutputLimits(1100, 1900);
  myController.setBias(255.0 / 2.0);
  myController.setWindUpLimits(-10, 10);
  myController.start();
  Serial.println("PID Controller ready");

  Serial.println("Starting Ping initialization...");
  while (!ping.initialize()) {
        Serial.println("\nPing device failed to initialize!");
        Serial.println("Are the Ping rx/tx wired correctly?");
        Serial.print("Ping rx is the green wire, and should be connected to Arduino pin ");
        Serial.print(arduinoTxPin);
        Serial.println(" (Arduino tx)");
        Serial.print("Ping tx is the white wire, and should be connected to Arduino pin ");
        Serial.print(arduinoRxPin);
        Serial.println(" (Arduino rx)");
        delay(2000);
  }
  Serial.println("Ping armed.");
  
  // Attach each ESC to its corresponding pin and send the stop signal
  Serial.println("Starting ESC initialization...");
  for (int i = 0; i < 4; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].writeMicroseconds(1500); // send "stop" signal to ESC to arm it
    Serial.print("ESC ");
    Serial.print(i);
    Serial.print(" attached to pin ");
    Serial.print(servoPins[i]);
    Serial.println(" with stop signal (1500 µs)");
  }
  delay(7000); // delay to allow the ESCs to recognize the stopped signal and arm
  Serial.println("ESCs armed.");
}



void loop() {
  if (ping.update()) {
        int dist = ping.distance();
        int conf = ping.confidence();

        if (conf >= CONFIDENCE_THRESH) input = dist / FACTOR;
    } else {
        Serial.println("No update received!");
    }

  myController.compute();
  // myController.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
  //                                             PRINT_OUTPUT   | // in the Serial plotter
  //                                             PRINT_SETPOINT
  //                                             // PRINT_BIAS     |
  //                                             // PRINT_P        |
  //                                             // PRINT_I        |
  //                                             // PRINT_D
  //                                             );

  //  Clockwise
  servos[2].writeMicroseconds(output); // Pin 6
  servos[3].writeMicroseconds(output); // Pin 10

  //  Counter Clockwise
  int diff = 1500 - output;
  int inverse_output = 1500 + diff;
  servos[1].writeMicroseconds(inverse_output); // Pin 5
  servos[0].writeMicroseconds(inverse_output); // Pin 3
  
  Serial.print("CW PWM:");
  Serial.print(output);
  Serial.print(",CCW PWM:");
  Serial.println(inverse_output);
}


#include <Servo.h>
#include "ArduPID.h"
#include "ping1d.h"
#include "SoftwareSerial.h"

#define SPEED_MULTIPLIER 0.15

const byte servoPins[] = {3, 5, 6, 10}; // signal pins for the four ESCs
Servo servos[4];

static const uint8_t arduinoRxPin = 11;
static const uint8_t arduinoTxPin = 9;
SoftwareSerial pingSerial = SoftwareSerial(arduinoRxPin, arduinoTxPin);
static Ping1D ping { pingSerial };

ArduPID myController;

const int MIN_DEPTH = 400; // in mm, how close the submarine can be to the bottom of the pool
const int MAX_DEPTH = 600; // in mm, how far the auv can be from the bottom of the pool, set this to avoid jumping out the pool

const float CONFIDENCE_THRESH = 100; // Remove for later
const float FACTOR = 4.55; // Remove for later (4.55 for air)

double input;
double output;

// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 800;
double p = 33.434711290669;
double i = 4.00673201254504;
double d = 42.0130886342262;

void setup() {
  pingSerial.begin(9600);
  Serial.begin(115200); // Faster serial communication for real-time debugging

  Serial.println("Starting PID Controller initialization...");
  myController.begin(&input, &output, &setpoint, p, i, d);
  myController.setOutputLimits(-SPEED_MULTIPLIER, SPEED_MULTIPLIER);
  // myController.setBias(255.0 / 2.0);
  // myController.setWindUpLimits(-10, 10);
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
  myController.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT
                                              // PRINT_BIAS     |
                                              // PRINT_P        |
                                              // PRINT_I        |
                                              // PRINT_D
                                              );

  Serial.print("Distance: ");
  Serial.println(input);
  Serial.print("Thrust Control Output:");
  Serial.println(output);

  int pwm = 1500 + 400 * output;

  //  Clockwise
  servos[2].writeMicroseconds(pwm); // Pin 6
  servos[3].writeMicroseconds(pwm); // Pin 10

  //  Counter Clockwise
  int diff = 1500 - pwm;
  int inverse_pwm = 1500 + diff;
  servos[1].writeMicroseconds(inverse_pwm); // Pin 5
  servos[0].writeMicroseconds(inverse_pwm); // Pin 3
  
  Serial.print("CW PWM:");
  Serial.print(pwm);
  Serial.print(",CCW PWM:");
  Serial.println(inverse_pwm);
}

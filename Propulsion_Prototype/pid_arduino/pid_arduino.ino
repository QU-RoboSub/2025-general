#include <Servo.h>
#include "ping1d.h"
#include "SoftwareSerial.h"

#define SPEED_MULTIPLIER 0

const byte servoPins[] = {3, 5, 6, 10};
Servo servos[4];

static const uint8_t arduinoRxPin = 11;
static const uint8_t arduinoTxPin = 9;
SoftwareSerial pingSerial = SoftwareSerial(arduinoRxPin, arduinoTxPin);
static Ping1D ping { pingSerial };

//const int MIN_DEPTH = 550; // Minimum depth limit in mm
//const int MAX_DEPTH = 650; // Maximum depth limit in mm

const float CONFIDENCE_THRESH = 100;
const float FACTOR = 1;

// PID Control Parameters
const float KP = 33.434711290669 / 200000;
const float KI = 0.000;
const float KD = 0;
float integralError = 0.0;
const float INTEGRAL_LIMIT = 100.0;
double previousError = 0;

double input;
double output;
double setpoint = 975;

void setup() {
  pingSerial.begin(9600);
  Serial.begin(115200);

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
    servos[i].writeMicroseconds(1500); // Send "stop" signal (1500 µs) to arm ESC
    Serial.print("ESC ");
    Serial.print(i);
    Serial.print(" attached to pin ");
    Serial.print(servoPins[i]);
    Serial.println(" with stop signal (1500 µs)");
  }
  delay(7000); // Delay to allow ESCs to recognize stop signal and arm
  Serial.println("ESCs armed.");
}

void loop() {
  if (ping.update()) {
    int dist = ping.distance();
    int conf = ping.confidence();

    if (conf >= CONFIDENCE_THRESH) {
      input = dist / FACTOR;
    }
  } else {
    Serial.println("No update received!");
  }

  // PID calculations
  double error = setpoint - input;

  // Proportional term
  float proportional = KP * error;

  // Integral term (with windup protection)
  integralError += error;
  integralError = constrain(integralError, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float integral = KI * integralError;

  // Derivative term
  float derivative = KD * (error - previousError);
  previousError = error;

  // Total control output
  output = proportional + integral + derivative;
  output = constrain(output, -0.1, 0.1);

  // Constrain output to within PWM limits
  int pwmValue = 1500 + 400 * output;
  int inverse_pwm = 3000 - pwmValue;

  if (error < 1500) {

    // Clockwise thrusters
    servos[0].writeMicroseconds(pwmValue); // Control ESC on pin 6
    servos[1].writeMicroseconds(pwmValue); // Control ESC on pin 10

    // Counter Clockwise thrusters
    servos[2].writeMicroseconds(inverse_pwm); // Control ESC on pin 5
    servos[3].writeMicroseconds(inverse_pwm); // Control ESC on pin 3

  }

  // Print debug information
  Serial.print("Distance: ");
  Serial.println(input);
  Serial.print("Thrust Control Output: ");
  Serial.println(output);
  Serial.print("CW PWM: ");
  Serial.print(pwmValue);
  Serial.print(", CCW PWM: ");
  Serial.println(inverse_pwm);
}

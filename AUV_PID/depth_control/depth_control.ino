#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

// Define PWM parameters
const int pwmPins[] = {27, 26, 25, 33, 32, 5, 18, 19}; // GPIO pins for PWM
const int pwmChannelBase = 0; // Base PWM channel (one for each pin)
const int pwmFrequency = 50;  // Frequency in Hz
const int pwmResolution = 10; // Resolution in bits (10-bit for more precise control)
float desiredDutyPercentage = 7.497; // Initial duty cycle percentage
int dutyCycle;

// Pin indices for movement (names swapped to match actual direction)
const int BACK_PIN1 = 2;     // Index for pin 25
const int BACK_PIN2 = 4;     // Index for pin 32
const int FORWARD_PIN1 = 5;  // Index for pin 5
const int FORWARD_PIN2 = 1;  // Index for pin 26
const int U1 = 0;     // Index for pin 27
const int U2 = 3;     // Index for pin 33
const int U3 = 6;     // Index for pin 18
const int U4 = 7;     // Index for pin 19


// Center point for duty cycle calculations
const float CENTER_DUTY = 7.5;

// PID Control Parameters
const float KP = -33.434711290669 / 200;
const float KI = 0.000;
const float KD = 0;
float integralError = 0.0;
const float INTEGRAL_LIMIT = 100.0;
double previousError = 0;

double input;
double output;
double setpoint = 0.3;

double depthOffset = 0;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Enter command for movement:");
  Serial.println("Examples:");
  Serial.println("  f8.5  -> forward");
  Serial.println("  b8.5  -> backward");
  Serial.println("  r8    -> right");
 
  // // Loop through all pins and configure PWM
  // for (int i = 0; i < sizeof(pwmPins) / sizeof(pwmPins[0]); i++) {
  //   int channel = pwmChannelBase + i;
  //   ledcSetup(channel, pwmFrequency, pwmResolution);
  //   ledcAttachPin(pwmPins[i], channel);
  // }
 
  // // Set initial duty cycle
  // updateAllThrusters(desiredDutyPercentage);

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);

}



void loop() {
  sensor.read();
  input = sensor.depth() - depthOffset;

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
  float depth_control = CENTER_DUTY + 2 * output;

  Serial.print("Current_Depth:");
  Serial.print(input);
  Serial.print(",");
  Serial.print("Target_Depth:");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print("Output:");
  Serial.print(output);
  Serial.print(",");
  Serial.print("Depth_Control:");
  Serial.println(depth_control);

  updateVSymmetricThrusters(depth_control);
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read input until newline
   
    // Check for forward or backward
    if (command.startsWith("f") || command.startsWith("b") || command.startsWith("r") || command.startsWith("l") || command.startsWith("v") || command.startsWith("y")) {
      float dutyCycleValue = command.substring(1).toFloat();
     
      // Validate duty cycle range (same logic as original)
      if ((!command.startsWith("v") && dutyCycleValue >= 5.5 && dutyCycleValue <= 9.5) || command.startsWith("v")) {
        // Calculate the mirrored duty cycle
        float difference = dutyCycleValue - CENTER_DUTY;
        float mirroredDuty = CENTER_DUTY - difference;
       
        if (command.startsWith("f")) {
          updateFBSymmetricThrusters(dutyCycleValue, mirroredDuty, true);
          Serial.print("Forward thrusters (pins 5, 26) duty cycle: ");
          Serial.print(dutyCycleValue);
          Serial.print("% | Backward thrusters (pins 25, 32) duty cycle: ");
          Serial.print(mirroredDuty);
          Serial.println("%");

        } else if (command.startsWith("b")){   // backward movement
          updateFBSymmetricThrusters(dutyCycleValue, mirroredDuty, false);
          Serial.print("Backward thrusters (pins 25, 32) duty cycle: ");
          Serial.print(dutyCycleValue);
          Serial.print("% | Forward thrusters (pins 5, 26) duty cycle: ");
          Serial.print(mirroredDuty);
          Serial.println("%");

        } else if(command.startsWith("r")) {
          updateRLSymmetricThrusters(dutyCycleValue, mirroredDuty, true);
          Serial.print("Right thrusters (pins 32, 5) duty cycle: ");
          Serial.print(dutyCycleValue);
          Serial.print("% | Left thrusters (pins 25, 26) duty cycle: ");
          Serial.print(mirroredDuty);
          Serial.println("%");

        } else if(command.startsWith("l")) {
          updateRLSymmetricThrusters(dutyCycleValue, mirroredDuty, false);
          Serial.print("Left thrusters (pins 25, 26) duty cycle: ");
          Serial.print(dutyCycleValue);
          Serial.print("% | Left thrusters (pins 32, 5) duty cycle: ");
          Serial.print(mirroredDuty);
          Serial.println("%");

        } else if(command.startsWith("v")){
          setpoint = dutyCycleValue;
        }

        else if(command.startsWith("y")){
          updateYSymmetricThrusters(dutyCycleValue, mirroredDuty);
        }
     
      }
      else {
        Serial.println("Invalid duty cycle. Please enter a value between 7.7% and 9.5%.");
      }
    }
    else if (command == "k") {
      updateAllThrusters(desiredDutyPercentage);
    }
    //  Start: take depth reading at air for offset, call this before starting to go underwaterx
    else if (command == "s") {
      depthOffset += input;
    }
    else {
      Serial.println("Invalid command. Use format: f<duty_cycle>, b<duty_cycle>, or r<duty_cycle>");
    }
  }
}

// Function to update thrusters symmetrically (forward/backward)
void updateFBSymmetricThrusters(float primaryDuty, float mirroredDuty, bool isForward) {
  int primaryDutyCycle = (int)((primaryDuty / 100.0) * ((1 << pwmResolution) - 1));
  int mirroredDutyCycle = (int)((mirroredDuty / 100.0) * ((1 << pwmResolution) - 1));
 
  if (isForward) {
    // Forward movement - primary duty to forward pins, mirrored to back pins
    ledcWrite(pwmChannelBase + FORWARD_PIN1, primaryDutyCycle);  // Pin 5
    ledcWrite(pwmChannelBase + FORWARD_PIN2, primaryDutyCycle);  // Pin 26
    ledcWrite(pwmChannelBase + BACK_PIN1, mirroredDutyCycle);    // Pin 25
    ledcWrite(pwmChannelBase + BACK_PIN2, mirroredDutyCycle);    // Pin 32
  } else {
    // Backward movement - primary duty to back pins, mirrored to forward pins
    ledcWrite(pwmChannelBase + BACK_PIN1, primaryDutyCycle);     // Pin 25
    ledcWrite(pwmChannelBase + BACK_PIN2, primaryDutyCycle);     // Pin 32
    ledcWrite(pwmChannelBase + FORWARD_PIN1, mirroredDutyCycle); // Pin 5
    ledcWrite(pwmChannelBase + FORWARD_PIN2, mirroredDutyCycle); // Pin 26
  }
}

// Function to update thrusters for right turn
void updateRLSymmetricThrusters(float primaryDuty, float mirroredDuty, bool isRight) {
  int primaryDutyCycle = (int)((primaryDuty / 100.0) * ((1 << pwmResolution) - 1));
  int mirroredDutyCycle = (int)((mirroredDuty / 100.0) * ((1 << pwmResolution) - 1));
 
  if (isRight){
    // According to the requirement:
    // Pins 32 and 5 get the "primaryDuty"
    // Pins 25 and 26 get the "mirroredDuty"
    ledcWrite(pwmChannelBase + BACK_PIN2, primaryDutyCycle);    // Pin 32
    ledcWrite(pwmChannelBase + FORWARD_PIN1, primaryDutyCycle); // Pin 5
    ledcWrite(pwmChannelBase + BACK_PIN1, mirroredDutyCycle);   // Pin 25
    ledcWrite(pwmChannelBase + FORWARD_PIN2, mirroredDutyCycle);// Pin 26
  } else {
    ledcWrite(pwmChannelBase + BACK_PIN1, primaryDutyCycle);    // Pin 32
    ledcWrite(pwmChannelBase + FORWARD_PIN2, primaryDutyCycle); // Pin 5
    ledcWrite(pwmChannelBase + BACK_PIN2, mirroredDutyCycle);   // Pin 25
    ledcWrite(pwmChannelBase + FORWARD_PIN1, mirroredDutyCycle);// Pin 26
  }
}


void updateVSymmetricThrusters(float primaryDuty) {
  int primaryDutyCycle = (int)((primaryDuty / 100.0) * ((1 << pwmResolution) - 1));
  ledcWrite(pwmChannelBase + U1, primaryDutyCycle);    // Pin 32
  ledcWrite(pwmChannelBase + U2, primaryDutyCycle); // Pin 5
  ledcWrite(pwmChannelBase + U3, primaryDutyCycle);   // Pin 25
  ledcWrite(pwmChannelBase + U4, primaryDutyCycle);// Pin 26

}

void updateYSymmetricThrusters(float primaryDuty, float mirroredDuty) {
  int primaryDutyCycle = (int)((primaryDuty / 100.0) * ((1 << pwmResolution) - 1));
  int mirroredDutyCycle = (int)((mirroredDuty / 100.0) * ((1 << pwmResolution) - 1));

  ledcWrite(pwmChannelBase + BACK_PIN2, primaryDutyCycle);    // Pin 32
  ledcWrite(pwmChannelBase + FORWARD_PIN2, primaryDutyCycle); // Pin 5
  ledcWrite(pwmChannelBase + BACK_PIN1, mirroredDutyCycle);   // Pin 25
  ledcWrite(pwmChannelBase + FORWARD_PIN1, mirroredDutyCycle);// Pin 26

}


// Function to update all thrusters (for initialization or other needs)
void updateAllThrusters(float percentage) {
  dutyCycle = (int)((percentage / 100.0) * ((1 << pwmResolution) - 1));
 
  for (int i = 0; i < sizeof(pwmPins) / sizeof(pwmPins[0]); i++) {
    ledcWrite(pwmChannelBase + i, dutyCycle);
  }
}
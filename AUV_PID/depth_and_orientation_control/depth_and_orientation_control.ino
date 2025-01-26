#include <Wire.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

// Depth sensor library
#include "MS5837.h"

// IMU libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>

MS5837 depthSensor;

Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

// Define PWM parameters
const int pwmPins[] = {27, 26, 25, 33, 32, 5, 18, 19}; // GPIO pins for PWM
const int pwmChannelBase = 0; // Base PWM channel (one for each pin)
const int pwmFrequency = 50;  // Frequency in Hz
const int pwmResolution = 10; // Resolution in bits (10-bit for more precise control)
float desiredDutyPercentage = 7.497; // Initial duty cycle percentage
int dutyCycle;

// Pin indices for movement (names swapped to match actual direction)
const int BACK_PIN1 = 7;     // Index for pin 25
const int BACK_PIN2 = 0;     // Index for pin 32
const int FORWARD_PIN1 = 4;  // Index for pin 5
const int FORWARD_PIN2 = 3;  // Index for pin 26
const int U1 = 6;     // Index for pin 27
const int U2 = 5;     // Index for pin 33
const int U3 = 1;     // Index for pin 18
const int U4 = 2;     // Index for pin 19

// Center point for duty cycle calculations
const float CENTER_DUTY = 7.5;

// Thrust allocation matrix
// TODO: Calculate and calibrate based on distance, location, orientation, etc.
// TODO: Precompute pseudoinverse
BLA::Matrix<6, 8> allocationMatrix = {
//27, 26, 25, 33, 32,  5, 18, 19
  -1,  0,  0,  1,  1,  0,  0, -1 // X axis allocation
   1,  0,  0, -1,  1,  0,  0, -1 // y axis allocation
   0,  1,  1,  0,  0,  1,  1,  0 // z axis allocation
};

// Control matrix where all PID outputs are stored
BLA::Matrix<6, 1> controlMatrix;

// Thruster duty cycles
BLA::Matrix<8, 1> thrusterInputMatrix;

// Depth Control Parameters and Values
float zP = -1.67173556453345;
float zI = 0.000;
float zD = 0;
float zIE = 0.0; // Integral error
float zPE = 0; // Previous error
float zIn;
float zOut;
float zTarget = 0.3;
float zOffset = 0;

// Roll Control Parameters and Values
float rP = -1.67173556453345;
float rI = 0.000;
float rD = 0;
float rIE = 0.0; // Integral error
float rPE = 0; // Previous error
float rIn;
float rOut;
float rTarget = 0;

// Pitch Control Parameters and Values
float pP = -1.67173556453345;
float pI = 0.000;
float pD = 0;
float pIE = 0.0; // Integral error
float pPE = 0; // Previous error
float pIn;
float pOut;
float pTarget = 0;

// Yaw Control Parameters and Values
float yP = -1.67173556453345;
float yI = 0.000;
float yD = 0;
float yIE = 0.0; // Integral error
float yPE = 0; // Previous error
float yIn;
float yOut;
float yTarget = 0;

// Safety Parameters
int thrustLimit = 1;
const float INTEGRAL_LIMIT = 100.0;

// Debug Parameters
bool zDebug = false;
bool rDebug = false;
bool pDebug = false;
bool yDebug = false;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Enter command for movement:");
  Serial.println("Examples:");
  Serial.println("  f8.5  -> forward");
  Serial.println("  b8.5  -> backward");
  Serial.println("  r8    -> right");
 
  // Loop through all pins and configure PWM
  for (int i = 0; i < sizeof(pwmPins) / sizeof(pwmPins[0]); i++) {
    int channel = pwmChannelBase + i;
    ledcSetup(channel, pwmFrequency, pwmResolution);
    ledcAttachPin(pwmPins[i], channel);
  }
 
  // Set initial duty cycle
  updateAllThrusters(desiredDutyPercentage);

  // Initialize IMU
  if(!accel.begin())
  {
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!depthSensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  depthSensor.setModel(MS5837::MS5837_30BA);
  depthSensor.setFluidDensity(997);

}

void loop() {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  // Read depth data from sensor
  depthSensor.read();
  zIn = depthSensor.depth() - zOffset;

  //  Calculate all 4 (later 6) PIDs
  calculatePID(zIn, zOut, zTarget, zP, zI, zD, zIE, zPE); // Depth (z)
  calculatePID(rIn, rOut, rTarget, rP, rI, rD, rIE, rPE); // Roll (r)
  calculatePID(pIn, pOut, pTarget, pP, pI, pD, pIE, pPE); // Pitch (p)
  calculatePID(yIn, yOut, yTarget, yP, yI, yD, yIE, yPE); // Yaw (y)

  // Combine output from all PID controllers into one control array
  BLA::Matrix<6, 1> controlMatrix = {
    0,
    0,
    zOut,
    rOut,
    pOut,
    yOut,
  };

  // Allocate PID outputs to different thrusters based on allocation matrix (thrust allocation)
  thrusterInputMatrix = allocationMatrix * controlMatrix;

  // Send thrust commands to all thrusters
  for (int i = 0; i < 8; i++) runThruster(i, T(i));

  if (zDebug) {
    Serial.println("Depth Control Params:");
    Serial.print("P: ");
    Serial.print(zP);
    Serial.print(", ");
    Serial.print("I: ");
    Serial.print(zI);
    Serial.print(", ");
    Serial.print("D: ");
    Serial.print(zD);
    Serial.print(", ");
    Serial.print("In: ");
    Serial.print(zIn);
    Serial.print(", ");
    Serial.print("Target: ");
    Serial.print(zTarget);
    Serial.print(", ");
    Serial.print("Out: ");
    Serial.println(zOut);
  }

  if (rDebug) {
    Serial.println("Roll Control Params:");
    Serial.print("P: ");
    Serial.print(rP);
    Serial.print(", ");
    Serial.print("I: ");
    Serial.print(rI);
    Serial.print(", ");
    Serial.print("D: ");
    Serial.print(rD);
    Serial.print(", ");
    Serial.print("In: ");
    Serial.print(rIn);
    Serial.print(", ");
    Serial.print("Target: ");
    Serial.print(rTarget);
    Serial.print(", ");
    Serial.print("Out: ");
    Serial.println(rOut);
  }

  if (pDebug) {
    Serial.println("Pitch Control Params:");
    Serial.print("P: ");
    Serial.print(pP);
    Serial.print(", ");
    Serial.print("I: ");
    Serial.print(pI);
    Serial.print(", ");
    Serial.print("D: ");
    Serial.print(pD);
    Serial.print(", ");
    Serial.print("In: ");
    Serial.print(pIn);
    Serial.print(", ");
    Serial.print("Target: ");
    Serial.print(pTarget);
    Serial.print(", ");
    Serial.print("Out: ");
    Serial.println(pOut);
  }

  if (yDebug) {
    Serial.println("Yaw Control Params:");
    Serial.print("P: ");
    Serial.print(yP);
    Serial.print(", ");
    Serial.print("I: ");
    Serial.print(yI);
    Serial.print(", ");
    Serial.print("D: ");
    Serial.print(yD);
    Serial.print(", ");
    Serial.print("In: ");
    Serial.print(yIn);
    Serial.print(", ");
    Serial.print("Target: ");
    Serial.print(yTarget);
    Serial.print(", ");
    Serial.print("Out: ");
    Serial.println(yOut);
  }

  // TODO: Refactor to have specific functions for forward, left, right, etc
  // TODO: Add commands to change PIDs for imu
  // if (Serial.available() > 0) {
  //   String command = Serial.readStringUntil('\n'); // Read input until newline
   
  //   // Check for forward or backward
  //   if (command.startsWith("f") || command.startsWith("b") || command.startsWith("r") || command.startsWith("l") || command.startsWith("v") || command.startsWith("y") || command.startsWith("p") || command.startsWith("i") || command.startsWith("d")) {
  //     float dutyCycleValue = command.substring(1).toFloat();
     
  //     // Validate duty cycle range (same logic as original)
  //     if ((!command.startsWith("v") && dutyCycleValue >= 5.5 && dutyCycleValue <= 9.5) || command.startsWith("v") || command.startsWith("p") || command.startsWith("i") || command.startsWith("d")) {
  //       // Calculate the mirrored duty cycle
  //       float difference = dutyCycleValue - CENTER_DUTY;
  //       float mirroredDuty = CENTER_DUTY - difference;
       
  //       if (command.startsWith("f")) {
  //         updateFBSymmetricThrusters(dutyCycleValue, mirroredDuty, true);
  //         Serial.print("Forward thrusters (pins 5, 26) duty cycle: ");
  //         Serial.print(dutyCycleValue);
  //         Serial.print("% | Backward thrusters (pins 25, 32) duty cycle: ");
  //         Serial.print(mirroredDuty);
  //         Serial.println("%");

  //       } else if (command.startsWith("b")){   // backward movement
  //         updateFBSymmetricThrusters(dutyCycleValue, mirroredDuty, false);
  //         Serial.print("Backward thrusters (pins 25, 32) duty cycle: ");
  //         Serial.print(dutyCycleValue);
  //         Serial.print("% | Forward thrusters (pins 5, 26) duty cycle: ");
  //         Serial.print(mirroredDuty);
  //         Serial.println("%");

  //       } else if(command.startsWith("r")) {
  //         updateRLSymmetricThrusters(dutyCycleValue, mirroredDuty, true);
  //         Serial.print("Right thrusters (pins 32, 5) duty cycle: ");
  //         Serial.print(dutyCycleValue);
  //         Serial.print("% | Left thrusters (pins 25, 26) duty cycle: ");
  //         Serial.print(mirroredDuty);
  //         Serial.println("%");

  //       } else if(command.startsWith("l")) {
  //         updateRLSymmetricThrusters(dutyCycleValue, mirroredDuty, false);
  //         Serial.print("Left thrusters (pins 25, 26) duty cycle: ");
  //         Serial.print(dutyCycleValue);
  //         Serial.print("% | Left thrusters (pins 32, 5) duty cycle: ");
  //         Serial.print(mirroredDuty);
  //         Serial.println("%");
  //       }

  //       //  Set new target depth
  //       else if(command.startsWith("v")){
  //         setpoint = dutyCycleValue;
  //       }
  //       //  Set P constant
  //       else if(command.startsWith("p")){
  //         zP = dutyCycleValue;
  //       }
  //       //  Set I constant
  //       else if(command.startsWith("i")){
  //         zI = dutyCycleValue;
  //       }
  //       //  Set D constant
  //       else if(command.startsWith("d")){
  //         zD = dutyCycleValue;
  //       }

  //       else if(command.startsWith("y")){
  //         updateYSymmetricThrusters(dutyCycleValue, mirroredDuty);
  //       }
     
  //     }
  //     else {
  //       Serial.println("Invalid duty cycle. Please enter a value between 7.7% and 9.5%.");
  //     }
  //   }
  //   else if (command == "k") {
  //     updateAllThrusters(desiredDutyPercentage);
  //     thrustLimit = 0;
  //   }
  //   else if (command == "t") {
  //     thrustLimit = 1;
  //   }
  //   //  Start: take depth reading at air for offset, call this before starting to go underwaterx
  //   else if (command == "s") {
  //     zOffset += input;
  //   }
  //   else {
  //     Serial.println("Invalid command. Use format: f<duty_cycle>, b<duty_cycle>, or r<duty_cycle>");
  //   }
  // }
}

void calculatePID(float &input, float &output, float &target, float p, float i, float d, float &integralError, float &prevError) {
  // Calculate error
  float error = target - input;

  // Proportional term
  float proportional = p * error;

  // Integral term (with windup protection)
  integralError += error;
  integralError = constrain(integralError, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float integral = i * integralError;

  // Derivative term
  float derivative = d * (error - prevError);
  prevError = error;

  // Total control output
  output = proportional + integral + derivative;
  output = constrain(output, -thrustLimit, thrustLimit);

  // Limit output if error is too high
  if (error > 1) output = 0;
}

void runThruster(int index, float signal) {
  // Make sure signal is from -1 to 1 and convert to duty cycle
  float duty = CENTER_DUTY + 2 * constrain(signal, -1.0, 1.0);
  int dutyCycle = (int)((duty / 100.0) * ((1 << pwmResolution) - 1));
  ledcWrite(pwmChannelBase + index, dutyCycle);
}

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

// Temp sensor library
#include "max6675.h"

MS5837 depthSensor;

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t   orientation;
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

int thermoDO = 4;
int thermoCS = 2;
int thermoCLK = 15;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

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
BLA::Matrix<8, 6> allocationMatrix = {
  -0.250000000000000,  0.250000000000000,  6.93889390390722e-18,  2.08166817117217e-17,  1.38777878078145e-17, -0.250000000000000,
    0.000000000000000,  0.000000000000000, -0.250000000000000,  0.250000000000000, -0.250000000000000,  1.38777878078145e-17,
    0.000000000000000,  0.000000000000000, -0.250000000000000, -0.250000000000000, -0.250000000000000,  0.000000000000000,
    0.250000000000000, -0.250000000000000, -2.54426109809932e-17,  4.39463280580791e-17, -4.62592926927149e-18, -0.250000000000000,
    0.250000000000000,  0.250000000000000,  1.61907524424502e-17,  2.31296463463574e-18,  2.31296463463574e-17,  0.250000000000000,
    0.000000000000000,  0.000000000000000, -0.250000000000000,  0.250000000000000,  0.250000000000000,  0.000000000000000,
    0.000000000000000,  0.000000000000000, -0.250000000000000, -0.250000000000000,  0.250000000000000,  0.000000000000000,
  -0.250000000000000, -0.250000000000000, -1.61907524424502e-17,  2.54426109809932e-17,  4.62592926927149e-18,  0.250000000000000
};
BLA::Matrix<6, 8> originalAllocationMatrix = {
//27, 26, 25, 33, 32,  5, 18, 19
  -1,  0,  0,  1,  1,  0,  0, -1, // X axis allocation
   1,  0,  0, -1,  1,  0,  0, -1, // y axis allocation
   0, -1, -1,  0,  0, -1, -1,  0, // z axis allocation
   0,  1, -1,  0,  0,  1, -1,  0, // roll axis allocation
   0, -1, -1,  0,  0,  1,  1,  0, // pitch axis allocation
  -1,  0,  0, -1,  1,  0,  0,  1  // yaw axis allocation
};

// Inertial control matrix where all PID outputs are stored
BLA::Matrix<6, 1> inertialControlMatrix;

// Control matrix in the body frame
BLA::Matrix<6, 1> bodyControlMatrix;

// Rotation matrix to transform control matrix from inertial frame to body frame
BLA::Matrix<3, 3> rotationMatrix;

// Thruster duty cycles
BLA::Matrix<8, 1> thrusterInputMatrix;

// Surge Control Parameters and Values
float xIE, xPE, xIn, xOut, xTarget;
float xP = 1;
float xI = 0;
float xD = 0;
float xOffset = 0;

// Sway Control Parameters and Values
float yIE, yPE, yIn, yOut, yTarget;
float yP = 1;
float yI = 0;
float yD = 0;
float yOffset = 0;

// Heave Control Parameters and Values
float zIE, zPE, zIn, zOut, zTarget;
float zP = 1.67173556453345;
float zI = 0;
float zD = 0;
float zOffset = 0;

// Roll Control Parameters and Values
float rIE, rPE, rIn, rOut, rTarget;
float rP = -0.01;
float rI = 0;
float rD = 0;
float rOffset = 111.5;

// Roll Kalman Filter Variables
static float rollEst = 0.0;
static float P_roll  = 1.0;
float Q_roll = 1;
float R_roll = 0.0003;

// Pitch Control Parameters and Values
float pIE, pPE, pIn, pOut, pTarget;
float pP = -0.01;
float pI = 0;
float pD = 0;
float pOffset = 34.5;

// Pitch Kalman Filter Variables
static float pitchEst = 0.0;
static float P_pitch = 1.0;
float Q_pitch = 1;
float R_pitch = 0.0003;

// Yaw Control Parameters and Values
float wIE, wPE, wIn, wOut, wTarget;
float wP = 0; // TODO: change when using cameras IMU
float wI = 0;
float wD = 0;
float wOffset = 0;

// Yaw Kalman Filter Variables
static float yawEst = 0.0;
static float P_yaw = 1.0;
float Q_yaw = 1;
float R_yaw = 0.0003;

// Safety Parameters
int thrustLimit = 0;
const int MAX_COMMANDS = 10;
const float INTEGRAL_LIMIT = 100.0;

// Debug Parameters
bool thrusterDebug = false;
bool tempDebug = true;
bool xDebug = false;
bool yDebug = false;
bool zDebug = false;
bool rDebug = false;
bool pDebug = false;
bool wDebug = false;

template <int rows, int cols>
BLA::Matrix<cols, rows> pseudoInverse(BLA::Matrix<rows, cols>& A) {
  BLA::Matrix<cols, rows> AT = ~A;
  BLA::Matrix<cols, cols> ATA = AT * A;
  BLA::Matrix<cols, cols> ATA_inv = Invert(ATA);
  return ATA_inv * AT;
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  Wire.begin();
 
  // Loop through all pins and configure PWM
  for (int i = 0; i < sizeof(pwmPins) / sizeof(pwmPins[0]); i++) {
    int channel = pwmChannelBase + i;
    ledcSetup(channel, pwmFrequency, pwmResolution);
    ledcAttachPin(pwmPins[i], channel);
  }

  // Set initial duty cycle
  for (int i = 0; i < 8; i++) runThruster(i, 0);

  delay(2000);

  // Initialize IMU
  if(!accel.begin()) Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
  if(!mag.begin()) Serial.println("Ooops, no LSM303 detected ... Check your wiring!");

  delay(2000);

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

  Serial.println("AUV Initialized");
}

void loop() {
  if (accel.begin() && mag.begin()) {
    // Get sensor events
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);

    // Get orientation from Adafruit's fusion algorithm
    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
      // Raw sensor fusion values
      float rawRoll = orientation.roll;
      float rawPitch = orientation.pitch;
      float rawYaw = orientation.heading;

      // Update Kalman filters for each angle
      calculateKalman(rollEst,    P_roll,    rawRoll,    Q_roll,    R_roll);
      calculateKalman(pitchEst,   P_pitch,   rawPitch,   Q_pitch,   R_pitch);
      calculateKalman(yawEst,     P_yaw,     rawYaw,     Q_yaw,     R_yaw);

      pIn = rollEst - pOffset;
      rIn = pitchEst - rOffset;
      wIn = yawEst - wOffset;
    }
  }

  // Read depth data from sensor
  if (depthSensor.init()) {
    depthSensor.read();
    zIn = depthSensor.depth() - zOffset;
  }

  //  Calculate all 6 PIDs
  // calculatePID(xIn, xOut, xTarget, xP, xI, xD, xIE, xPE, false); // Surge (x)
  // calculatePID(yIn, yOut, yTarget, yP, yI, yD, yIE, yPE, false); // Sway (y)
  calculatePID(zIn, zOut, zTarget, zP, zI, zD, zIE, zPE, false); // Depth (z)
  calculatePID(rIn, rOut, rTarget, rP, rI, rD, rIE, rPE, true); // Roll (r)
  calculatePID(pIn, pOut, pTarget, pP, pI, pD, pIE, pPE, true); // Pitch (p)
  calculatePID(wIn, wOut, wTarget, wP, wI, wD, wIE, wPE, true); // Yaw (w)

  // Combine output from all PID controllers into one control array
  inertialControlMatrix = {
    xOut,
    yOut,
    zOut,
    rOut,
    pOut,
    yOut,
  };

  // Apply orientation matrix to control matrix to get body frame
  calculateRotationMatrix();
  convertInertialToBodyMatrix();

  // Allocate PID outputs to different thrusters based on allocation matrix (thrust allocation)
  thrusterInputMatrix = allocationMatrix * bodyControlMatrix;

  // Send thrust commands to all thrusters
  for (int i = 0; i < 8; i++) runThruster(i, thrusterInputMatrix(i));

  if (thrusterDebug) {
    Serial.println("Thruster Output Matrix:");
    Serial.println(thrustLimit);
    for (int i = 0; i < 8; i++) Serial.print(String(thrusterInputMatrix(i)) + ", ");
    Serial.println('\n');
  }

  if (tempDebug) {
    Serial.print("Temperature Reading: "); 
    Serial.print(thermocouple.readCelsius());
    Serial.println(" C"); 
  }

  if (xDebug) debugParameters("Surge", xP, xI, xD, xIn, xTarget, xOut);
  if (yDebug) debugParameters("Sway", yP, yI, yD, yIn, yTarget, yOut);
  if (zDebug) debugParameters("Heave", zP, zI, zD, zIn, zTarget, zOut);
  if (rDebug) debugParameters("Roll", rP, rI, rD, rIn, rTarget, rOut);
  if (pDebug) debugParameters("Pitch", pP, pI, pD, pIn, pTarget, pOut);
  if (wDebug) debugParameters("Yaw", wP, wI, wD, wIn, wTarget, wOut);

  // Allow user to change variable values during runtime
  if (Serial.available() > 0) parseCommands(Serial.readStringUntil('\n'));
}

// Parses a list of commands from the serial monitor
void parseCommands(String input) {
  while (input.length() > 0) {
    // Take the next command until ;
    int separatorIndex = input.indexOf(';');
    String command = (separatorIndex == -1) ? input : input.substring(0, separatorIndex);

    // Split individual command into varName and newValue
    String varName, newValue;
    int splitIndex = input.indexOf('=');
    if (splitIndex == -1) varName = input;
    else {
      varName = input.substring(0, splitIndex);
      newValue = input.substring(splitIndex + 1);
    }

    // General functions
    if (varName == "kill") thrustLimit = 0;
    else if (varName == "arm") thrustLimit = 1;
    else if (varName == "thrusterDebug") thrusterDebug = !thrusterDebug;
    else if (varName == "tempDebug") tempDebug = !tempDebug;

    // Change axes parameters
    if (varName.startsWith("x")) changeParameters(varName, newValue, xP, xI, xD, xIn, xTarget, xOut, xOffset, xDebug);
    else if (varName.startsWith("y")) changeParameters(varName, newValue, yP, yI, yD, yIn, yTarget, yOut, yOffset, yDebug);
    else if (varName.startsWith("z")) changeParameters(varName, newValue, zP, zI, zD, zIn, zTarget, zOut, zOffset, zDebug);
    else if (varName.startsWith("r")) changeParameters(varName, newValue, rP, rI, rD, rIn, rTarget, rOut, rOffset, rDebug);
    else if (varName.startsWith("p")) changeParameters(varName, newValue, pP, pI, pD, pIn, pTarget, pOut, pOffset, pDebug);
    else if (varName.startsWith("w")) changeParameters(varName, newValue, wP, wI, wD, wIn, wTarget, wOut, wOffset, wDebug);

    // Error case
    else {
      Serial.println("Make sure input is in format <varName>=<newValue>");
      Serial.println(varName + " is not an accepted variable");
    }

    // Remove processed part
    input = (separatorIndex == -1) ? "" : input.substring(separatorIndex + 1);
  }
}

// Takes in varName=newValue and figures out which variable to change, works for all 6 DOFs
void changeParameters(String varName, String newValue, float &p, float &i, float &d, float &in, float &target, float &out, float &offset, bool &debug) {
  if (varName.endsWith("P")) p = newValue.toFloat();
  else if (varName.endsWith("I")) i = newValue.toFloat();
  else if (varName.endsWith("D")) d = newValue.toFloat();
  else if (varName.endsWith("In")) in = newValue.toFloat();
  else if (varName.endsWith("Target")) target = newValue.toFloat();
  else if (varName.endsWith("Out")) out = newValue.toFloat();
  else if (varName.endsWith("Debug")) debug = !debug;

  else if (varName.endsWith("Offset")) {
    bool isLinear = varName.startsWith("x") || varName.startsWith("y") || varName.startsWith("z");
    if (isLinear) offset = in;
    else {
      offset = in;
      if (offset < -180) offset += 360;
      else if (offset > 180) offset -= 360;
    }
  }
}

// Used to debug parameters and output to serial monitor
void debugParameters(String axis, float &p, float &i, float &d, float &in, float &target, float &out) {
  Serial.printf("%s Control Params:", axis);
  Serial.print("P: ");
  Serial.print(p);
  Serial.print(", ");
  Serial.print("I: ");
  Serial.print(i);
  Serial.print(", ");
  Serial.print("D: ");
  Serial.print(d);
  Serial.print(", ");
  Serial.print("In: ");
  Serial.print(in);
  Serial.print(", ");
  Serial.print("Target: ");
  Serial.print(target);
  Serial.print(", ");
  Serial.print("Out: ");
  Serial.println(out);
}

// Apply kalman filter to IMU data to get better readings
void calculateKalman(float& x_est, float& P_est, float z_meas, float Q, float R) {
  // 1. Predict step
  float x_pred = x_est;     // No state evolution in this simple model
  float P_pred = P_est + Q; // Add process noise

  // 2. Update step
  float y = z_meas - x_pred;    // Innovation
  float S = P_pred + R;         // Innovation covariance
  float K = P_pred / S;         // Kalman gain

  // New state estimate
  x_est = x_pred + K * y;

  // New error covariance
  P_est = (1.0f - K) * P_pred;
}

// Create rotation matrix based on current IMU data
void calculateRotationMatrix() {
  // Precalculate cos and sin of each angle in radians
  float cr = cos((rIn * 71.0) / 4068.0);
  float sr = sin((rIn * 71.0) / 4068.0);
  float cp = cos((pIn * 71.0) / 4068.0);
  float sp = sin((pIn * 71.0) / 4068.0);
  float cw = cos((wIn * 71.0) / 4068.0);
  float sw = sin((wIn * 71.0) / 4068.0);

  // https://ae640a.github.io/assets/winter17/slides/Lecture%203.pdf (page 11)
  rotationMatrix = {
    cp*cw,              cp*sw,              -sp,
    sr*sp*cw - cr*sw,   sr*sp*sw + cr*cw,   sr*cp,
    cr*sp*cw + sr*sw,   cr*sp*sw - sr*cw,   cr*cp
  };
}

// Converts control signal matrix in 6 DOFs to thruster commands for all 8 thrusters
void convertInertialToBodyMatrix() {
  // Transform linear control signals (first 3 entries)
  BLA::Matrix<3, 1> ctrlLinear = {inertialControlMatrix(0), inertialControlMatrix(1), inertialControlMatrix(2)};
  BLA::Matrix<3, 1> transformedLinear = rotationMatrix * ctrlLinear;

  bodyControlMatrix(0) = transformedLinear(0);
  bodyControlMatrix(1) = transformedLinear(1);
  bodyControlMatrix(2) = transformedLinear(2);

  // Copy rotational signals directly (last 3 entries)
  bodyControlMatrix(3) = inertialControlMatrix(3);
  bodyControlMatrix(4) = inertialControlMatrix(4);
  bodyControlMatrix(5) = inertialControlMatrix(5);
}

// Main PID controller logic
void calculatePID(float &input, float &output, float &target, float p, float i, float d, float &integralError, float &prevError, bool isAngle) {
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

  // If this is for an angular axis, make sure the error is within -180 to 180
  // For e.g, if in=10 and target=350, error should be 20, not 340
  // For e.g, if in=350 and target=10, error should be 20, not -340
  if (isAngle) {
    if (error < -180) error += 360;
    else if (error > 180) error -= 360;
  }

  // // Limit output if error is too high
  // if (error > 1) output = 0;
}

// Runs a specific thruster at the given control signal (-1 to 1 where 1 is full thrust forward)
void runThruster(int index, float signal) {
  // Make sure signal is from -1 to 1 and convert to duty cycle
  float duty = CENTER_DUTY + 2 * (signal * thrustLimit);
  int dutyCycle = (int)((duty / 100.0) * ((1 << pwmResolution) - 1));
  ledcWrite(pwmChannelBase + index, dutyCycle);
}

const int HALL_SENSOR_PIN = A0; // Analog input for Hall effect sensor
const int PWM_PIN = 6; // PWM output for H-Bridge control

// Control parameters
const int MIN_DISTANCE = 50; // Closest safe distance
const int MAX_DISTANCE = 200; // Farthest operational distance
const float KP = 1.8; // Proportional gain
const float KI = 1.63; // Integral gain
const float KD = 0.5; // Derivative gain

// Variables for digital filtering
int previousPwmValue = 0;
const int MAX_PWM_CHANGE = 10; // Limit rapid PWM changes for stability

// PID control variables
float integralError = 0.0; // Accumulated integral error
const float INTEGRAL_LIMIT = 100.0; // Limit integral windup
int previousError = 0; // Store the previous error for derivative calculation

void setup() {
  Serial.begin(115200); // Faster serial communication for real-time debugging
  pinMode(HALL_SENSOR_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
}

int readHallSensor() {
  long totalReading = 0;
  const int NUM_SAMPLES = 4;
  
  for (int i = 0; i < NUM_SAMPLES; i++) {
    totalReading += analogRead(HALL_SENSOR_PIN);
  }
  
  int avgReading = totalReading / NUM_SAMPLES;
  return constrain(avgReading, MIN_DISTANCE, MAX_DISTANCE);
}

int calculateMagneticStrength() {
  int distance = readHallSensor();
  int targetDistance = (MIN_DISTANCE + MAX_DISTANCE) / 2; // Set desired levitation point
  
  // Calculate error (invert logic)
  int error = distance - targetDistance; // Inverted: more force if magnet is farther
  
  // Proportional term
  float proportional = KP * error;

  // Integral term (with windup protection)
  integralError += error;
  integralError = constrain(integralError, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float integral = KI * integralError;

  // Derivative term
  float derivative = KD * (error - previousError);
  previousError = error; // Update previous error

  // Total control output
  int pwmValue = proportional + integral + derivative;
  
  // Constrain PWM value
  pwmValue = constrain(pwmValue, 0, 255);
  
  // Implement soft ramping to prevent sudden changes
  int pwmDiff = pwmValue - previousPwmValue;
  if (abs(pwmDiff) > MAX_PWM_CHANGE) {
    pwmValue = previousPwmValue + (pwmDiff > 0 ? MAX_PWM_CHANGE : -MAX_PWM_CHANGE);
  }
  
  previousPwmValue = pwmValue; // Update previous PWM value
  return pwmValue;
}

void loop() {
  int magneticStrength = calculateMagneticStrength();
  analogWrite(PWM_PIN, magneticStrength);
  
  #ifdef DEBUG
  Serial.print("PWM: ");
  Serial.println(magneticStrength);
  #endif
}

Kind Regards,
Muhammad Putra 
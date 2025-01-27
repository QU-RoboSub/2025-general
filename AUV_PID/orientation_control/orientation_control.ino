#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

// --------------------------------------------------------------------------
// Kalman filter variables for Roll
static float rollEst = 0.0;   // Filtered roll estimate
static float P_roll  = 1.0;   // Error covariance
float Q_roll = 0.0001;         // Process noise (tune as needed)
float R_roll = 0.03;          // Measurement noise (tune as needed)

// Kalman filter variables for Pitch
static float pitchEst = 0.0;  // Filtered pitch estimate
static float P_pitch = 1.0;   // Error covariance
float Q_pitch = 0.001;        // Process noise (tune as needed)
float R_pitch = 0.03;         // Measurement noise (tune as needed)

// Kalman filter variables for Heading
static float headingEst = 0.0; // Filtered heading estimate
static float P_heading = 1.0;  // Error covariance
float Q_heading = 0.001;       // Process noise (tune as needed)
float R_heading = 0.05;        // Measurement noise (tune as needed)

// --------------------------------------------------------------------------
// 1D Kalman filter update function
void kalmanUpdate(float& x_est, float& P_est, float z_meas, float Q, float R)
{
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

// --------------------------------------------------------------------------
void setup(void)
{
  Serial.begin(115200);
  // Comment out or remove if you want ONLY numeric data:
  // Serial.println("Adafruit 9 DOF with 1D Kalman Filters");

  if(!accel.begin())
  {
    Serial.println("Ooops, no LSM303 accel detected ... Check your wiring!");
    while(1);
  }
  if(!mag.begin())
  {
    Serial.println("Ooops, no LSM303 mag detected ... Check your wiring!");
    while(1);
  }
}

// --------------------------------------------------------------------------
void loop(void)
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  // Get sensor events
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  // Get orientation from Adafruit's fusion algorithm
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    // Raw sensor fusion values
    float rawRoll = orientation.roll;
    float rawPitch = orientation.pitch;
    float rawHeading = orientation.heading;

    // Update Kalman filters for each angle
    kalmanUpdate(rollEst,    P_roll,    rawRoll,    Q_roll,    R_roll);
    kalmanUpdate(pitchEst,   P_pitch,   rawPitch,   Q_pitch,   R_pitch);
    kalmanUpdate(headingEst, P_heading, rawHeading, Q_heading, R_heading);

    // ----------------------------------------------------------------------
    // Send just the filtered angles to the Serial Plotter (tab separated):
    Serial.print(rollEst);
    Serial.print("\t");
    Serial.print(pitchEst);
    Serial.print("\t");
    Serial.println(headingEst);

    // If you want to view raw vs filtered in the console (NOT for Plotter):
    // Serial.print("Raw: ");    Serial.print(rawRoll); Serial.print(", ");
    //                           Serial.print(rawPitch); Serial.print(", ");
    //                           Serial.print(rawHeading); Serial.print(" | ");
    // Serial.print("Filtered: "); Serial.print(rollEst); Serial.print(", ");
    //                             Serial.print(pitchEst); Serial.print(", ");
    //                             Serial.println(headingEst);
  }

  delay(100);
}

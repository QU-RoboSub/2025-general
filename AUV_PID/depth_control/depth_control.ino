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

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
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

}



void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input until newline
   
    // Check for forward or backward
    if (input.startsWith("f") || input.startsWith("b") || input.startsWith("r") || input.startsWith("l") || input.startsWith("v") || input.startsWith("y")) {
      float dutyCycleValue = input.substring(1).toFloat();
     
      // Validate duty cycle range (same logic as original)
      if (dutyCycleValue >= 5.5 && dutyCycleValue <= 9.5) {
        // Calculate the mirrored duty cycle
        float difference = dutyCycleValue - CENTER_DUTY;
        float mirroredDuty = CENTER_DUTY - difference;
       
        if (input.startsWith("f")) {
          updateFBSymmetricThrusters(dutyCycleValue, mirroredDuty, true);
          Serial.print("Forward thrusters (pins 5, 26) duty cycle: ");
          Serial.print(dutyCycleValue);
          Serial.print("% | Backward thrusters (pins 25, 32) duty cycle: ");
          Serial.print(mirroredDuty);
          Serial.println("%");

        } else if (input.startsWith("b")){   // backward movement
          updateFBSymmetricThrusters(dutyCycleValue, mirroredDuty, false);
          Serial.print("Backward thrusters (pins 25, 32) duty cycle: ");
          Serial.print(dutyCycleValue);
          Serial.print("% | Forward thrusters (pins 5, 26) duty cycle: ");
          Serial.print(mirroredDuty);
          Serial.println("%");

        } else if(input.startsWith("r")) {
          updateRLSymmetricThrusters(dutyCycleValue, mirroredDuty, true);
          Serial.print("Right thrusters (pins 32, 5) duty cycle: ");
          Serial.print(dutyCycleValue);
          Serial.print("% | Left thrusters (pins 25, 26) duty cycle: ");
          Serial.print(mirroredDuty);
          Serial.println("%");

        } else if(input.startsWith("l")) {
          updateRLSymmetricThrusters(dutyCycleValue, mirroredDuty, false);
          Serial.print("Left thrusters (pins 25, 26) duty cycle: ");
          Serial.print(dutyCycleValue);
          Serial.print("% | Left thrusters (pins 32, 5) duty cycle: ");
          Serial.print(mirroredDuty);
          Serial.println("%");

        } else if(input.startsWith("v")){
          updateVSymmetricThrusters(dutyCycleValue);
        }

        else if(input.startsWith("y")){
          updateYSymmetricThrusters(dutyCycleValue, mirroredDuty);
        }
     
      }
      else {
        Serial.println("Invalid duty cycle. Please enter a value between 7.7% and 9.5%.");
      }
    }
    else if (input == "k") {
      updateAllThrusters(desiredDutyPercentage);
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
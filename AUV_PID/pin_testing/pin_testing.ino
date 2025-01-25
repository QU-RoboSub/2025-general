#include <Arduino.h>

#define LEDC_FREQ 50    // 50 kHz frequency
#define LEDC_CHANNEL 0     // LEDC channel (0-15)
#define LEDC_RESOLUTION 10 // 10-bit resolution
#define DUTY_CYCLE 77      // 7.5% duty cycle (out of 1023 for 10-bit resolution)

void setup() {
  Serial.begin(115200);
  Serial.println("Enter pin number for PWM output:");
}

void loop() {
  if (Serial.available() > 0) {
    int pin = Serial.parseInt();

    // Configure LEDC for the selected pin
    ledcSetup(LEDC_CHANNEL, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(pin, LEDC_CHANNEL);
    ledcWrite(LEDC_CHANNEL, DUTY_CYCLE);
    delay(2000);
    ledcWrite(LEDC_CHANNEL, 85);
    delay(5000);
    ledcWrite(LEDC_CHANNEL, DUTY_CYCLE);
    
    Serial.print("PWM output configured on pin ");
    Serial.println(pin);
  }
}
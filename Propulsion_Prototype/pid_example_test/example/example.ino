#include "ArduPID.h"
#include "ping1d.h"
#include "SoftwareSerial.h" 

static const uint8_t arduinoRxPin = 11;
static const uint8_t arduinoTxPin = 9;
SoftwareSerial pingSerial = SoftwareSerial(arduinoRxPin, arduinoTxPin);
static Ping1D ping { pingSerial };

const float CONFIDENCE_THRESH = 0; // Remove for later
const float FACTOR = 4.55; // Remove for later

ArduPID myController;




double input;
double output;

// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 500;
double p = 33.43;
double i = 4;
double d = 42.01;




void setup() {
  Serial.begin(115200);

  pingSerial.begin(9600);

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
 
  myController.begin(&input, &output, &setpoint, p, i, d);

  // myController.reverse()               // Uncomment if controller output is "reversed"
  // myController.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  myController.setOutputLimits(-1, 1);
  myController.setBias(255.0 / 2.0);
  myController.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
  
  myController.start();
  // myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
  // myController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)
}




void loop()
{
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
  
  // analogWrite(3, output); // Replace with plant control signal
}

#include "C:\Users\research_bu\Documents\GitHub\2025-general\Propulsion_Prototype\BR_Ping.h"
#include "arduino.h"
#include "ping1d.h"
#include "SoftwareSerial.h"

// arduinoRxPin uint8 [1,1] Non tunable
// arduinoTxPin uint8 [1,1] Non tunable
// ledPin uint8 [1,1] Non tunable

SoftwareSerial pingSerial;
static Ping1D ping { pingSerial };

void setupFunctionBR_Ping(uint8_T  arduinoRxPin,int size_vector__1,uint8_T  arduinoTxPin,int size_vector__2,uint8_T  ledPin,int size_vector__3){
    pingSerial = SoftwareSerial(arduinoRxPin, arduinoTxPin);
    pingSerial.begin(9600);
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    Serial.println("Blue Robotics ping1d-simple.ino");
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
}

// Distance int8 [1,1]


void stepFunctionBR_Ping(int8_T * Distance,int size_vector_1){
    if (ping.update()) {
        Distance = ping.distance();
        //Confidence = ping.confidence();
    } else {
        Serial.println("No update received!");
    }
}
#include "C:\Users\research_bu\Documents\GitHub\2025-general\Propulsion_Prototype\Ping_v3.h"

#include "arduino.h"
#include "ping1d.h"
#include "SoftwareSerial.h"

#define RXPIN 11
#define TXPIN 9
SoftwareSerial pingSerial = SoftwareSerial(RXPIN, TXPIN);
static Ping1D ping { pingSerial };

void setupFunctionPing_v3(){
    pingSerial.begin(9600);
    Serial.begin(115200);
    Serial.println("Blue Robotics ping1d-simple.ino");
    while (!ping.initialize()) {
        Serial.println("\nPing device failed to initialize!");
        Serial.println("Are the Ping rx/tx wired correctly?");
        Serial.print("Ping rx is the green wire, and should be connected to Arduino pin ");
        Serial.print(RXPIN);
        Serial.println(" (Arduino tx)");
        Serial.print("Ping tx is the white wire, and should be connected to Arduino pin ");
        Serial.print(TXPIN);
        Serial.println(" (Arduino rx)");
        delay(2000);
    }
}

// Distance uint32 [1,1]
// Confidence uint16 [1,1]


void stepFunctionPing_v3(uint32_T * Distance,int size_vector_1,uint16_T * Confidence,int size_vector_2){
  if (ping.update()) {
        Distance = (uint32_T*) ping.distance();
        Confidence = (uint16_T*) ping.confidence();
    } else {
        Serial.println("No update received!");
    }
}
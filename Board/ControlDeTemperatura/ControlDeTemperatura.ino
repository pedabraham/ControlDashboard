#include <ArduinoJson.h>
#include "lmt01.h"
const size_t capacity = JSON_OBJECT_SIZE(7) + 20;
DynamicJsonDocument doc(capacity);

LMT01 lmt;

ISR(ANALOG_COMP_vect)
{
  // // switch on debug LED to measure ISR time
  PORTB |= (1<<5);
  lmt.ISRCorrection();
  PORTB &= ~(1<<5);
}

void setup() {
 // initialize serial communications at 9600 bps:
    Serial.begin(115200);
    lmt.setup(12);
    lmt.setSingleMode();
}
float a0 = 0; // 0.434343
float a1 = 0; // 9990.43
float a2 = 0; // 9439393904.43
int b0 = 0;
float ref = 0; // 434349340.43
int yk1 = 0;
int yk = 0;
float ek_1 = 0;
float ek_2 = 0;
float ek = 0.755;
float temp = 0.0;


void loop() {

delay(500);

    if (Serial.available() > 0) {
        deserializeJson(doc, Serial);
        delay(100);
        if (doc["p"]==0){
            a0 = float(doc["a0"]); // 0.434343
            a1 = float(doc["a1"]);
            Serial.print("a0:");
            Serial.print(a0,5);
            Serial.print(" a1:");
            Serial.println(a1,5);
        }
        
        else if (doc["p"]==1) {
            a2 = float(doc["a2"]); // 9439393904.43
            ref = int(doc["ref"]); // 434349340.43
            b0 = int(doc["b0"]);
            Serial.print(" ref:");
            Serial.print(ref);
            Serial.print(" a2:");
            Serial.print(a2,5);
            Serial.print(" b0:");
            Serial.println(b0,5);
            
        }
    }

    temp = lmt.getSingleMeasure();
    
    yk = b0 * yk1 + a0 * ek + a1 * ek_1 + a2 * ek_2;
    yk1 = yk;
    ek_2 = ek_1;
    ek_1 = ek;


}
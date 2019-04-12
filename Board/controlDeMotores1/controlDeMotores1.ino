#include <ArduinoJson.h>
const size_t capacity = JSON_OBJECT_SIZE(7) + 20;
DynamicJsonDocument doc(capacity);
void setup() {
  Serial.begin(115200);
  pinMode(6, OUTPUT);
}
float a0 = 0; // 0.434343
float a1 = 0; // 9990.43
float a2 = 0; // 9439393904.43
int b0 = 0;
float ref = 0; // 434349340.43
float yk1 = 0;
float yk = 0;
float ek_1 = 0;
float ek_2 = 0;
float ek = 0;
float dato = 0.0;
int ts= 175;
bool flag = true;
void loop() {

    if (Serial.available() > 0) {
        deserializeJson(doc, Serial);
        delay(100);
        yk =0;
        yk1 = 0;
        ek_1 = 0;
        ek_2 = 0;
        if (doc["p"]==0){
            a0 = float(doc["a0"]); // 0.434343
            a1 = float(doc["a1"]);
            Serial.print(a0,5);

            if (flag)
            {
                flag= false;
            }
            else{
                flag = true;
            }
        }
        else if (doc["p"]==1) {
            a2 = float(doc["a2"]); // 9439393904.43
            ref = int(doc["ref"]); // 434349340.43
            b0 = int(doc["b0"]);
            if (flag)
            {
                flag= false;
            }
            else{
                flag = true;
            }
        }
    }
    if (flag){
        dato = analogRead(A0);
        //Serial.println(1);
        Serial.print(temp);
        ek = ref - temp;
        Serial.print(" ek: ");
        Serial.print(ek);
        yk = b0 * yk1 + a0 * ek + a1 * ek_1 + a2 * ek_2;
        //Serial.println(a0);
        Serial.print(" yk: ");
        Serial.println(yk);
        yk1 = yk;
        ek_2 = ek_1;
        ek_1 = ek;
        if (yk>255) {
            analogWrite(3,255);
        }else if (yk<1){
            analogWrite(3,0);
        }else{
            analogWrite(3,yk);
        }
        delay(ts);
    }
}

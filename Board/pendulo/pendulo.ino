#include <ArduinoJson.h>
const size_t capacity = JSON_OBJECT_SIZE(7) + 20;
DynamicJsonDocument doc(capacity);
int pwmP = 11;
void setup() {
  Serial.begin(115200);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(pwmP,OUTPUT);
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
int ts= 5;
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
            //Serial.println(a0);

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
        //Serial.println(flag);
        dato = analogRead(A0);
        //Serial.println(1);
        //Serial.print(dato);
        ek = ref - dato;
        //Serial.print(" ek: ");
        Serial.print(dato);
        Serial.print(" ");
        Serial.print(ek);
        
        yk = b0 * yk1 + a0 * ek + a1 * ek_1 + a2 * ek_2;

        //Serial.println(a0);
        Serial.print(" yk: ");
        Serial.println(yk);
        yk1 = yk;
        ek_2 = ek_1;
        ek_1 = ek;
        if (yk>255) {
            analogWrite(pwmP,255);
            
            //Serial.println("total");
        }else if (yk<1){
            analogWrite(pwmP,yk*-1);
            digitalWrite(12,HIGH);
            digitalWrite(13,LOW);
        }else{
            analogWrite(pwmP,yk);
            digitalWrite(12,LOW);
            digitalWrite(13,HIGH);
        }
        delay(5);
    }
}
 
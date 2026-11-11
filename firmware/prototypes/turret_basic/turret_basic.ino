#include <WiFi.h>
//reference ESP-NOW: https://docs.arduino.cc/tutorials/nano-esp32/esp-now/ 
#include <esp_now.h>
//reference controle servo: https://esp32io.com/tutorials/esp32-servo-motor
#include <ESP32Servo.h>

#define SERVO_PIN 18
#define LASER_PIN 19

Servo servoMotor;

//message object receive struct
typedef struct struct_message {
    char a[32]; //label
    float value;
    uint32_t x; //x
    uint32_t y; //y
    uint32_t width;
    uint32_t height;
} struct_message;

struct_message objectData;

//Toggle pin laser en utilisant un modulo 2 sur i
void pulseLaser(uint8_t pin){
  for(int i = 0; i<100; i++){
    digitalWrite(pin, i % 2);
    delay(15);
  }
}

void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData,
                int len) {

  memcpy(&objectData, incomingData, sizeof(objectData));
  Serial.print("Bytes received: ");
  Serial.println(len);

  Serial.println("From MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", info->src_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.printf("\n");
  Serial.print("Label: ");
  Serial.println(objectData.a);
  Serial.print("Value: ");
  Serial.println(objectData.value);
  Serial.print("X: ");
  Serial.println(objectData.x);
  Serial.print("Y: ");
  Serial.println(objectData.y);
  Serial.print("Width: ");
  Serial.println(objectData.width);
  Serial.print("Height: ");
  Serial.println(objectData.height);
  Serial.println();

  if(strcmp(objectData.a, "bug") == 0 && objectData.value > 0.5){
    //vise vers la gauche
    //TODO: Implementer vrai calcul
    if(objectData.x < 50){
      for (int pos = 0; pos <= 90; pos += 1) {
        servoMotor.write(pos);
        delay(15); // waits 15ms to reach the position
      }
    }
    //vers la droite
    //TODO: implementer vrai calcul
    else if(objectData.x > 50){
      for (int pos = 90; pos >= 0; pos -= 1) {
        servoMotor.write(pos);
        delay(15); // waits 15ms to reach the position
      }
    }
    //tir
    pulseLaser(LASER_PIN);
  }

}

void setup() {

  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }

  esp_now_register_recv_cb(OnDataRecv);

  servoMotor.attach(SERVO_PIN);
  
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
}

void loop() {

}


/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <WiFi.h>
//reference ESP-NOW: https://docs.arduino.cc/tutorials/nano-esp32/esp-now/ 
#include <esp_now.h>
//reference controle servo: https://esp32io.com/tutorials/esp32-servo-motor
#include <ESP32Servo.h>
#include <math.h>


#define DEV_I2C Wire

#define I2C_SCL 4
#define I2C_SDA 5

#define SERVO_X 18
#define SERVO_Y 19


#define SerialPort Serial

#ifndef LED_BUILTIN
  #define LED_BUILTIN 8
#endif
#define LedPin LED_BUILTIN

#define LASER 7
#define LIDARPOW 3

// Components.
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, -1);

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

Servo servoX;
Servo servoY;

struct ServoAngles {
  int panAngle;
  int tiltAngle;
};

ServoAngles finalAngles;

//----------- Dimensions en mm ----------
// const float LASER_H = 0; //hauteur du pointeur laser sur son mounting
// const float TURRET_H =0; //hauteur de tour
// const float CAMERA_H = 0; //de camera a top vehicule
// offset entre laser et camera
const float TURRET_OFFSET_X = -20.0;//20 mm gauche
const float TURRET_OFFSET_Y = 25.0; //mm en haut
const float TURRET_OFFSET_Z = 0.0;

// --- Camera Specifications ---
const float RES_X = 192.0;   // e.g., QVGA width
const float RES_Y = 192.0;   // e.g., QVGA height
const float CAM_FOV_X = 53.0;    // Horizontal FOV in degrees
const float CAM_FOV_Y = 40.0;    // Vertical FOV in degrees
 
const float DEG_TO_RAD1 = PI / 180.0;
const float RAD_TO_DEG1 = 180.0 / PI;

const int OFFSET_DEG_Y = 47;
const int OFFSET_DEG_X = -1;
 

// ------------------ FONCTIONS UTILES ------------------

// Function to call when Edge Impulse finds a bounding box
//ServoAngles calculateServoAngles(float boxCenterX, float boxCenterY, float distLidar) {
  
//   // 1. Convert pixel coordinates to Normalized Device Coordinates (-1.0 to 1.0)
//   // X: Negative is Left, Positive is Right
//   // Y: Negative is Down, Positive is Up (Note: Screen Y is usually inverted, 0 is top)
//   float nx = ((RES_X/ 2.0)-boxCenterX ) / (RES_X / 2.0);
//   float ny = (boxCenterY-(RES_Y/ 2.0)) / (RES_Y / 2.0); 
  
//   // 2. Convert to Camera Angles (in Radians)
//   float thetaX_rad = nx * (CAM_FOV_X / 2.0) * (PI / 180.0);
//   float thetaY_rad = ny * (CAM_FOV_Y / 2.0) * (PI / 180.0);

//   // 3. Project into 3D Space (Camera Origin)
//   float Xc = distLidar * sin(thetaX_rad) * cos(thetaY_rad);
//   float Yc = distLidar * sin(thetaY_rad);
//   float Zc = distLidar * cos(thetaX_rad) * cos(thetaY_rad);

//   // 4. Shift to Turret Origin
//   float Xt = Xc - TURRET_OFFSET_X;
//   float Yt = Yc + TURRET_OFFSET_Y; // Subtracting a positive value pushes the target "down" relative to the turret
//   float Zt = Zc - TURRET_OFFSET_Z;

//   // 5. Calculate Raw Turret Angles (in Radians)
//   float pan_rad = atan2(Xt, Zt);
  
//   float ground_dist = sqrt((Xt * Xt) + (Zt * Zt));
//   float tilt_rad = atan2(Yt, ground_dist);

//   // 6. Convert to Degrees and Map to Servo Physical Constraints
//   float raw_pan_deg = pan_rad * (180.0 / PI);
//   float raw_tilt_deg = tilt_rad * (180.0 / PI);

//   ServoAngles finalAngles;

//   // Pan: Math says 0 is straight ahead. Servo says 90 is straight ahead.
//   // (If your turret pans backward, change the '+' to a '-')
//   finalAngles.panAngle = round(90.0 - raw_pan_deg);

//   // Tilt: Math says negative is down. Servo says positive is down.
//   // We invert the sign so a target below the turret results in a positive servo angle.
//   finalAngles.tiltAngle = round(raw_tilt_deg);

//   // 7. Safety Clamps (Prevent servos from trying to rip the turret apart)
//   if (finalAngles.panAngle < 20) finalAngles.panAngle = 20;
//   if (finalAngles.panAngle > 160) finalAngles.panAngle = 160;
  
//   if (finalAngles.tiltAngle < 0) finalAngles.tiltAngle = 0;
//   if (finalAngles.tiltAngle > 90) finalAngles.tiltAngle = 90; // Assuming it shouldn't look past straight down

//   SerialPort.println(" Angles finaux");
//   SerialPort.println(finalAngles.tiltAngle); 
//   SerialPort.println(finalAngles.panAngle);
//   return finalAngles;
// }
// Function to call when Edge Impulse finds a bounding box
ServoAngles calculateServoAngles(float boxCenterX, float boxCenterY, float distLidar) {
 
  // 1. Calculate angles relative to camera (Accounts for Upside-Down Camera)
  // +dx means target is to the physical right. +dy means target is physically up.
  float angleCamX_deg = ((RES_X/ 2.0) - boxCenterX) / RES_X * CAM_FOV_X;
  float angleCamY_deg = (boxCenterY - (RES_Y/ 2.0)) / RES_Y * CAM_FOV_Y;
 
  // 2. Calculate physical 3D position of target relative to camera lens
  float targetX_cam = distLidar * tan(angleCamX_deg * DEG_TO_RAD1);
  float targetY_cam = distLidar * tan(angleCamY_deg * DEG_TO_RAD1);
 
  // 3. Translate to Laser's coordinate frame (Parallax correction)
  float targetX_laser = targetX_cam - TURRET_OFFSET_X;
  float targetY_laser = targetY_cam - TURRET_OFFSET_Y;
 
  // 4. Calculate raw angles needed for the laser/turret
  float panAngle_deg = atan2(targetX_laser, distLidar) * RAD_TO_DEG1;
  float tiltAngle_deg = atan2(targetY_laser, distLidar) * RAD_TO_DEG1;
 
  
 
  // Pan: Math says 0 is straight ahead. Servo says 90 is straight ahead.
  // (If your turret pans backward, change the '+' to a '-')
  finalAngles.panAngle = round(90.0 - panAngle_deg);
 
  // Tilt: Math says negative is down. Servo says positive is down.
  // We invert the sign so a target below the turret results in a positive servo angle.
  finalAngles.tiltAngle = round(tiltAngle_deg);
 
  // 7. Safety Clamps (Prevent servos from trying to rip the turret apart)
  if (finalAngles.panAngle < 0) finalAngles.panAngle = 0;
  if (finalAngles.panAngle > 180) finalAngles.panAngle = 180;
 
  if (finalAngles.tiltAngle < 0) finalAngles.tiltAngle = 0;
  if (finalAngles.tiltAngle > 90) finalAngles.tiltAngle = 90; // Assuming it shouldn't look past straight down
    SerialPort.println(" Angles finaux");
  SerialPort.println(finalAngles.tiltAngle); 
  SerialPort.println(finalAngles.panAngle);
 
  return finalAngles;
}

// Measure distance using lidar tof sensor Adafruit VL53L4CX

float measureDistance(){
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;
  char report[64];
  int status;
  do {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);
  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
    snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
    SerialPort.print(report);
    for (j = 0; j < no_of_object_found; j++) {
      if (j != 0) {
        SerialPort.print("\r\n                               ");
      }
      SerialPort.print("status=");
      SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
      SerialPort.print(", D=");
      SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
      SerialPort.print("mm");
      SerialPort.print(", Signal=");
      SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
      SerialPort.print(" Mcps, Ambient=");
      SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
      SerialPort.print(" Mcps");
    }
    SerialPort.println("");
    if (status == 0) {
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
    return pMultiRangingData->RangeData[0].RangeMilliMeter;
  }
}

/**
*
* @param
* 
*/
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

  // Calculer le centre de la boite d'objet
  float centerX = objectData.x + (objectData.width / 2.0);
  float centerY = objectData.y + (objectData.height / 2.0);
  
  float dist = measureDistance();
  ServoAngles target = calculateServoAngles(centerX, centerY, dist);
  servoX.write(target.panAngle+OFFSET_DEG_X);
  int angleY_Servo = target.tiltAngle+OFFSET_DEG_Y;
  servoY.write(angleY_Servo);
  SerialPort.println(angleY_Servo);
  // servoX.write(centerX/320*66);
  // servoY.write(20);
  digitalWrite(LASER, 1);
  // delay(500);
  // digitalWrite(LASER,0);
                }

/* Setup ---------------------------------------------------------------------*/

void setup()
{
  // Led.
  pinMode(LedPin, OUTPUT);
  pinMode(LIDARPOW, OUTPUT);
  pinMode(LASER, OUTPUT);
  digitalWrite(LIDARPOW, 0);
  delay(3000);
  digitalWrite(LIDARPOW, 1);

  // Initialize serial for output.
  SerialPort.begin(115200);
  SerialPort.println("Starting...");

  //ESP-NOW Init
  SerialPort.println("ESP-NOW init...");
  WiFi.mode(WIFI_MODE_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }
  esp_now_register_recv_cb(OnDataRecv);
  //servo controls
  SerialPort.println("Servo Init");
  servoX.attach(SERVO_X);
  servoY.attach(SERVO_Y);

  servoY.write(0); //init angle at 0deg
  servoX.write(90); //init in center
  for(int i=45; i<136; i++){
    servoX.write(i);
    delay(5);
  }
  for(int i=135; i>45; i--){
    servoX.write(i);
    delay(5);
  }
  servoX.write(90); //init in center
  for(int i=0; i<90; i++){
    servoY.write(i);
    delay(5);
  }
  for(int i=90; i>0; i--){
    servoY.write(i);
    delay(5);
  }
  servoY.write(0); //init angle at 0deg
  servoX.write(90); //init in center
    // Initialize I2C bus.
  DEV_I2C.begin(I2C_SDA, I2C_SCL);
  SerialPort.println("Wire init");

  // Configure VL53L4CX satellite component.

  sensor_vl53l4cx_sat.begin();
  SerialPort.println("begin ok");

  // Switch off VL53L4CX satellite component.
  sensor_vl53l4cx_sat.VL53L4CX_Off();
  SerialPort.println("sat off ok");

  //Initialize VL53L4CX satellite component.
  sensor_vl53l4cx_sat.InitSensor(0x29);
  SerialPort.println("sat i2c init ok");

  // Start Measurements
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
  SerialPort.println("start measures");

  
  
}

void loop()
{

}

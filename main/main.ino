//include libraries
#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <arduino-timer.h>
#include <ArduinoJson.h>

//sensor
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//i2c
#include <Wire.h>

#define WEBSERVER_H
// pin definitions
#define PPL1 2
#define PPL2 4
#define GPSTX 17
#define GPSRX 16
#define SD_CS 5
#define SD_SCK 18
#define SD_MOSI 23
#define SD_MISO 19
#define Analog1 35
#define Analog2 34
#define TestLED 21
#define Button 22
#define I2C_SDA 33
#define I2C_SCL 32
#define Servo1 25
#define Servo2 26
#define Servo3 27
#define Servo4 14

#define ONBOARD_LED 2


// global vars
bool buttonPressed = false;
bool globalStatus = true;
float xrot;
float yrot;
float zrot;

float xaccel;
float yaccel;
float zaccel;


// servo definitions
Servo s1;
Servo s2;
Servo s3;
Servo s4;

//imu definition
Adafruit_MPU6050 imu;

//wifi settings
AsyncWebServer server(80);
const char* ssid = "LTG1 Aircraft Data Logger";
const char* password = "inh_aircraft";

//make timers
auto timer = timer_create_default();

void recvMsg(uint8_t *data, size_t len){
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}
void IRAM_ATTR onButtonPress(){
  //function called when button is pressed
  setServoPos(20,1);
  Serial.println("Button pressed");
}

bool onboard_led_blink(void *) {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // toggle the LED
  if(digitalRead(LED_BUILTIN) == HIGH){ //turn off again after 50ms
    timer.in(50,onboard_led_blink);
  }
  
  return true; // keep timer active? true
}

bool refreshSensors(void *){
  readIMU();
  return true;
}
void handleDataRequest(AsyncWebServerRequest *request){
  StaticJsonDocument<200> doc;
  JsonArray data = doc.createNestedArray("sensordata");

  JsonObject sensor1 = data.createNestedObject();
  sensor1["pressure"] = readAnalogInput();

  JsonObject sensor2 = data.createNestedObject();
  sensor2["temperature"] = random(10, 30);
  JsonObject imuData = data.createNestedObject();
  imuData["xrot"] = xrot;
  imuData["yrot"] = yrot;
  imuData["zrot"] = zrot;
  
  imuData["xaccel"] = xaccel;
  imuData["yaccel"] = yaccel;  
  imuData["zaccel"] = zaccel;

  String jsonString;
  serializeJson(doc,jsonString);
  request->send(200, "application/json", jsonString);

}


void initIMU(){
  if (!imu.begin()){
    Serial.println("IMU not found");
  }
  imu.setAccelerometerRange(MPU6050_RANGE_16_G);
  imu.setGyroRange(MPU6050_RANGE_250_DEG); //250 deg/s 
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}

void setup() {
  digitalWrite(TestLED, HIGH);
  
  Serial.begin(115200);

  //Networking  
  server.on("/json", HTTP_GET, handleDataRequest);
  configureWiFi();
  initIMU();

  //Set servos
  s1.attach(Servo1);
  s2.attach(Servo2);
  s3.attach(Servo3);
  s4.attach(Servo4);

  //Set outputs
  pinMode(TestLED, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);

  //Set inputs
  pinMode(Button, INPUT_PULLUP);
  pinMode(34, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL);

  //interrupts
  attachInterrupt(Button, onButtonPress, FALLING); //button pulls down, so detect falling edge for press  
  
  //timers
  timer.every(2500, onboard_led_blink);
  timer.every(5,refreshSensors);

  digitalWrite(TestLED, LOW);
}

void loop() {
  timer.tick();
}

void setServoPos(int position, int servoID) {
  if (position == 181) {
    position = 0;
  }

  if (servoID == 1) {
    s1.write(position);
  }
  if (servoID == 2) {
    s2.write(position);
  }
  if (servoID == 3) {
    s3.write(position);
  }
  if (servoID == 4) {
    s4.write(position);
  }


  //log the movement
  Serial.print("Servo ");
  Serial.print(servoID);
  Serial.print(" moved to ");
  Serial.println(position);
}

void printSeparator() {
  Serial.println("-----------------");
}

void configureWiFi(){
  //start wifi
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);
  /* Attach Message Callback */
  WebSerial.msgCallback(recvMsg);
  server.begin();
}


void readIMU() {
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);

  //imuData[0] = accel.acceleration.x;
  //imuData[1] = accel.acceleration.y;
  //imuData[2] = accel.acceleration.z;
  Serial.println(accel.acceleration.z);
  xrot = gyro.gyro.x;
  yrot = gyro.gyro.y;
  zrot = gyro.gyro.z;
  xaccel = accel.acceleration.x;
  yaccel = accel.acceleration.y;
  zaccel = accel.acceleration.z;



  //xaccel = accel.acceleration.z;
}

int readAnalogInput(){
  return analogRead(34);
}

bool switchLED(void *){
  digitalWrite(TestLED, !digitalRead(TestLED));
}


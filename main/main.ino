//include libraries
#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

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
#define SDA 33
#define SCL 32
#define Servo1 25
#define Servo2 26
#define Servo3 27
#define Servo4 14

#define ONBOARD_LED 2

// global vars
bool buttonPressed = false;
unsigned long currentTime = 0;

int LEDBlinkPeriod = 100;

// servo definitions
Servo s1;
Servo s2;
Servo s3;
Servo s4;

//wifi settings
AsyncWebServer server(80);
const char* ssid = "lt acft logger";
const char* password = "inh_aircraft";



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


void setup() {
  digitalWrite(TestLED, HIGH);
  
  Serial.begin(115200);
  //Set servos
  s1.attach(Servo1);
  s2.attach(Servo2);
  s3.attach(Servo3);
  s4.attach(Servo4);

  //Set outputs
  pinMode(21, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);

  //Set inputs
  pinMode(Button, INPUT_PULLUP);

  //interrupts
  attachInterrupt(Button, onButtonPress, FALLING); //button pulls down, so detect falling edge for press  configureWiFi();
  
  digitalWrite(TestLED, LOW);
}

void loop() {
  currentTime = millis();

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

void heartbeat(){

}


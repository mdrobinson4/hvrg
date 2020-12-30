#include <ArduinoJson.h>

#include <SerialCommand.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

SerialCommand SCmd;   // The demo SerialCommand object

#define TCAADDR 0x70

//Adafruit_BNO055 bno[3];
Adafruit_BNO055 bno[] = {Adafruit_BNO055(55, BNO055_ADDRESS_A), // upper arm
  Adafruit_BNO055(56, BNO055_ADDRESS_B), // forearm
  Adafruit_BNO055(57, BNO055_ADDRESS_A)}; // wrist

int bno_cnt = 1; //sizeof(bno) / sizeof(bno[0]);


void setup() {
  Serial.begin(9600);
  Serial.println("starting setup");
  for (int i = 0; i < bno_cnt; i++) {
    setup_bno(i);
  }
  Serial.println("setup done");

  // Setup callbacks for SerialCommand commands 
  SCmd.addCommand("update_pose", update_pose);
  SCmd.addDefaultHandler(unrecognized);  // Handler for command that isn't matched  (says "What?") 
}

void loop() {
  if (Serial.available() > 0) {
    SCmd.readSerial();
  }
}

void setup_bno(int i) {
  tcaselect(i);
  
  if(!bno[i].begin()) {
    Serial.print("Ooops, ");
    Serial.print(i);
    Serial.print(" not detected");
    while(1);
  }
  delay(1000); 
  bno[i].setExtCrystalUse(true);
}

void tcaselect(uint8_t i) {
  if (i > 7)
  return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void update_pose() {
  StaticJsonDocument<200> doc;
  for (int i = 0; i < bno_cnt; i++) {
    imu::Vector<3> euler;
    imu::Quaternion quat = bno[0].getQuat();
    JsonObject  data = doc.createNestedObject();
    euler = quat.toEuler();
    data["cnt"] = bno_cnt;
    data["x"] = quat.x();
    data["y"] = quat.y();
    data["z"] = quat.z();
    data["w"] = quat.w();
  }
  serializeJson(doc, Serial);
  Serial.println();
}

// This gets set as the default handler, and gets called when no other command matches. 
void unrecognized()
{
  //Serial.println("Invalid command"); 
}

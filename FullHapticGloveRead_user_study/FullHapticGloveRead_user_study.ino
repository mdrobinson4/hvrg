#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#define TCAADDR 0x70
#define pi 3.14159265

Adafruit_BNO055 bnoForearm = Adafruit_BNO055(55, BNO055_ADDRESS_A); 
Adafruit_BNO055 bnoUpperarm = Adafruit_BNO055(56, BNO055_ADDRESS_B); 
Adafruit_BNO055 bnoWrist = Adafruit_BNO055(57, BNO055_ADDRESS_A);

sensors_event_t event;

ros::NodeHandle nh;

std_msgs::Int16MultiArray finger_vals;
std_msgs::Int16 middle_vals;
geometry_msgs::Vector3 imuForearm;
geometry_msgs::Vector3 imuUpperarm;
geometry_msgs::Vector3 imuWrist;

geometry_msgs::Quaternion imuUpperarmQ;
geometry_msgs::Quaternion imuForearmQ;
geometry_msgs::Quaternion imuWristQ;

std_msgs::Int16 cal;

bool hapticFeedback;

int thumbZero = -90;
int thumbMax = 325;

int pointerZero = -30;
int pointerMax = -110;

int middleZero = 100;
int middleMax = 230;
int num;

//---------IMUS---------------//

ros::Publisher imuForearm_pub("imuForearm", &imuForearm);
ros::Publisher imuUpperarm_pub("imuUpperarm", &imuUpperarm);
ros::Publisher imuWrist_pub("imuWrist", &imuWrist);

ros::Publisher imuUpperarmQ_pub("imuUpperarmQ", &imuUpperarmQ);
ros::Publisher imuForearmQ_pub("imuForearmQ", &imuForearmQ);
ros::Publisher imuWristQ_pub("imuWristQ", &imuWristQ);

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void doFeedbackCB(const std_msgs::Bool& msg){
  hapticFeedback = msg.data;
}

ros::Subscriber<std_msgs::Bool> doHapticFeedback("doFeedback", &doFeedbackCB ); 


//--------ALL FINGERS---------------//

void allFingersCB( const std_msgs::Int16& msg){
  analogWrite(12, msg.data);
  analogWrite(9, msg.data);
  analogWrite(11, msg.data); 
}

ros::Subscriber<std_msgs::Int16> allFingersHaptic("haptic_all", &allFingersCB ); 

//--------THUMB---------------//

void hapticThumbCB( const std_msgs::Int16& msg){
  if(hapticFeedback){
    num = abs(msg.data - thumbZero);
    if(num < 40){
      num = 0;
    }
    else {
      num = map(num, 0, abs(thumbMax - thumbZero), 100, 255);
    }
    analogWrite(12, num); 
  }
  else{
    analogWrite(12, 0); 
  }

}

ros::Subscriber<std_msgs::Int16> force_thumb("force_thumb", &hapticThumbCB ); 

//---------POINTER----------//

void hapticPointerCB( const std_msgs::Int16& msg){
  if(hapticFeedback){
    num = abs(msg.data - pointerZero);
    if(num < 40){
      num = 0;
    }
    else {
      num = map(num, 0, abs(pointerMax-pointerZero), 100, 255);
    }
    analogWrite(9, num); 
  }
} 

ros::Subscriber<std_msgs::Int16> force_pointer("haptic_pointer", &hapticPointerCB ); 

//---------MIDDLE------------//

void hapticMiddleCB( const std_msgs::Int16& msg){
  if(hapticFeedback){
    num = abs(msg.data - middleZero);
    if(num < 40){
      num = 0;
    }
    else {
      num = map(num, 0, abs(middleMax-middleZero), 100, 255);
    }
    analogWrite(11, num);  
  }
  else{
    analogWrite(11, 0);
  }

}

ros::Subscriber<std_msgs::Int16> force_middle("haptic_middle", &hapticMiddleCB );

//--------CURVATURE SENSING------------//
ros::Publisher fingerPos("fingersPos", &finger_vals);
ros::Publisher middlePos("middlePos", &middle_vals);

unsigned long start;

void setup()
{
  pinMode(13, OUTPUT);

  pinMode(9, OUTPUT); //works
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);

  //Timer 0, 2, 3
  TCCR1B = TCCR1B & 0b11111000 | 0x05;
  TCCR2B = TCCR2B & 0b11111000 | 0x07;

  hapticFeedback = true;

  nh.initNode();

  nh.subscribe(doHapticFeedback);

  nh.subscribe(force_thumb);
  nh.subscribe(force_pointer);
  nh.subscribe(force_middle);

  nh.subscribe(allFingersHaptic);

  finger_vals.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  finger_vals.data_length = 5;

  finger_vals.layout.dim[0].label = "finger_vals";
  finger_vals.layout.dim[0].size = 5;
  finger_vals.layout.dim[0].stride = 1*5;
  finger_vals.layout.data_offset = 0;
  finger_vals.data = (int *)malloc(sizeof(int)*5);
  nh.advertise(fingerPos);

  nh.advertise(imuForearm_pub);
  nh.advertise(imuUpperarm_pub);
  nh.advertise(imuWrist_pub);

  nh.advertise(imuWristQ_pub);
  nh.advertise(imuForearmQ_pub);
  nh.advertise(imuUpperarmQ_pub);


  tcaselect(5);
  bnoForearm.begin();
  tcaselect(4);
  bnoWrist.begin();
  tcaselect(5);
  bnoUpperarm.begin();

  tcaselect(5); 
    if(!bnoForearm.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    nh.loginfo("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    else{ 
    nh.loginfo("#2 is all ok!"); 
  }
  delay(1000);
  bnoForearm.setExtCrystalUse(true);  
  
  tcaselect(4); 
    if(!bnoWrist.begin())
  {
    /* There was a problem detecting the BNO455 ... check your connections */
    nh.loginfo("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }  
  else{ 
    nh.loginfo("#1 is all ok!"); 
  }
  delay(1000);
  bnoWrist.setExtCrystalUse(true);

  tcaselect(5); 
    if(!bnoUpperarm.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    nh.loginfo("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    else{ 
    nh.loginfo("#3 is all ok!"); 
  }
  delay(1000);
  bnoForearm.setExtCrystalUse(true);
}

void loop()
{   

int thumbVal = analogRead(A2);
int pointerVal = analogRead(A1);
int middleVal = analogRead(A0);

finger_vals.data[0] = thumbVal;
finger_vals.data[1] = pointerVal;
finger_vals.data[2] = middleVal;
finger_vals.data[3] = middleVal;
finger_vals.data[4] = middleVal;
middle_vals.data = pointerVal;

fingerPos.publish( &finger_vals );
middlePos.publish( &middle_vals );

sensors_event_t event;
sensors_event_t event2;
sensors_event_t event3;

tcaselect(5);
imu::Quaternion Forearmquat = bnoForearm.getQuat();
imu::Vector<3> ForearmEuler = Forearmquat.toEuler();
imuForearm.x = ForearmEuler.x()*180/pi;
imuForearm.y = ForearmEuler.y()*180/pi;
imuForearm.z = ForearmEuler.z()*180/pi;

imuForearmQ.x = Forearmquat.x();
imuForearmQ.y = Forearmquat.y();
imuForearmQ.z = Forearmquat.z();
imuForearmQ.w = Forearmquat.w();

imuForearm_pub.publish( &imuForearm );
imuForearmQ_pub.publish( &imuForearmQ );

tcaselect(4);
imu::Quaternion Wristquat = bnoWrist.getQuat();
imu::Vector<3> WristEuler = Wristquat.toEuler();
imuWrist.x = WristEuler.x()*180/3.14159;
imuWrist.y = (WristEuler.y()*180/pi);
imuWrist.z = WristEuler.z()*180/pi;

imuWristQ.x = Wristquat.x();
imuWristQ.y = Wristquat.y();
imuWristQ.z = Wristquat.z();
imuWristQ.w = Wristquat.w();

imuWrist_pub.publish( &imuWrist );
imuWristQ_pub.publish( &imuWristQ );

tcaselect(5);
imu::Quaternion Upperquat = bnoUpperarm.getQuat();
imu::Vector<3> UpperEuler = Upperquat.toEuler();
imuUpperarm.x = UpperEuler.x()*180/3.14159;
imuUpperarm.y = UpperEuler.y()*180/pi;
imuUpperarm.z = UpperEuler.z()*180/pi;

imuUpperarmQ.x = Upperquat.x();
imuUpperarmQ.y = Upperquat.y();
imuUpperarmQ.z = Upperquat.z();
imuUpperarmQ.w = Upperquat.w();

imuUpperarm_pub.publish( &imuUpperarm );
imuUpperarmQ_pub.publish( &imuUpperarmQ );   

nh.spinOnce();
delay(10);

}

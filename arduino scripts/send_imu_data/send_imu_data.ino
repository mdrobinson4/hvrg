#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#define pi 3.14159265

geometry_msgs::Vector3 imuWristE;
geometry_msgs::Quaternion imuWristQ;
geometry_msgs::Pose wristPose;
geometry_msgs::Point wristPosition;
Adafruit_BNO055 bnoWrist = Adafruit_BNO055(55);

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;


ros::NodeHandle nh;

ros::Publisher imuWristE_pub("imuWristE", &imuWristE);
ros::Publisher imuWristQ_pub("imuWristQ", &imuWristQ);
ros::Publisher wristPose_pub("wristPose", &wristPose);
     
void setup(void) {
  nh.initNode();
  nh.advertise(imuWristE_pub);
  nh.advertise(imuWristQ_pub);
  nh.advertise(wristPose_pub);
  /* Initialise the sensor */
  if(!bnoWrist.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    nh.loginfo("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
    
  bool flag = false;
  while (flag == false) {
    if (!bnoWrist.begin()) {
      nh.loginfo("No BN0055 detected, kill yourself!");
    }
    else {
      nh.loginfo("Let's goo!!!");
      flag = true;
    }
  }
  bnoWrist.setExtCrystalUse(true);
}
     
void loop(void) {
  // get the quaternion (x,y,z,w) of the imu located on wrist
  imu::Quaternion imuWrist = bnoWrist.getQuat();
  getPosition();
  
  imuWristE = quat2eul(imuWrist);
  imuWristQ = getRosQ(imuWrist);
  wristPose.orientation = imuWristQ;
  wristPose.position = wristPosition;

  imuWristQ_pub.publish(&imuWristQ);   
  imuWristE_pub.publish(&imuWristE);   
  wristPose_pub.publish(&wristPose);
  nh.spinOnce();
  delay(10);
}

void getPosition() {
  sensors_event_t linearAccelData;
  bnoWrist.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  wristPosition.x = wristPosition.x + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  wristPosition.y = wristPosition.y + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
  wristPosition.z = wristPosition.z + ACCEL_POS_TRANSITION * linearAccelData.acceleration.z;
  
}

geometry_msgs::Quaternion getRosQ(imu::Quaternion imuWrist) {
  imuWristQ.x = imuWrist.x();
  imuWristQ.y = imuWrist.y();
  imuWristQ.z = imuWrist.z();
  imuWristQ.w = imuWrist.w();
  return imuWristQ;
}

geometry_msgs::Vector3 quat2eul(imu::Quaternion q) {
  imu::Vector<3> eul = q.toEuler();
  geometry_msgs::Vector3 imuEul;
  imuEul.x = eul.x() * (180 / pi);
  imuEul.y = eul.y() * (180 / pi);
  imuEul.z = eul.z() * (180 / pi);
}

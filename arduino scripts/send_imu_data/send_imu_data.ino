#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
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

//imu
Adafruit_BNO055 bnoWrist = Adafruit_BNO055(55);

// global variables to pusblish
geometry_msgs::Quaternion orientation;
geometry_msgs::Quaternion linearAcceleration;
geometry_msgs::Quaternion angularVelocity;
std_msgs::Int16MultiArray finger_vals;

// ros node
ros::NodeHandle nh;

// setup publishers
ros::Publisher orientation_pub("orientation", &orientation);
ros::Publisher linearAccel_pub("linear_accel", &linearAcceleration);
ros::Publisher angularVel_pub("angular_vel", &angularVelocity);
ros::Publisher fingerVals_pub("finger_vals", &finger_vals);
     
void setup(void) {
  nh.initNode();
  nh.advertise(orientation_pub);
  nh.advertise(linearAccel_pub);
  nh.advertise(angularVel_pub);

  // initialize finger array
  finger_vals.data_length = 5;
  finger_vals.layout.dim[0].label = "finger_vals";
  finger_vals.layout.dim[0].size = 5;
  finger_vals.layout.dim[0].stride = 1*5;
  finger_vals.layout.data_offset = 0;
  finger_vals.data = (int *)malloc(sizeof(int)*5);
  
  /* Initialise the sensor */
  if(!bnoWrist.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    nh.loginfo("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
    
  /* Initialise the sensor */
  if (!bnoWrist.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
  bnoWrist.setExtCrystalUse(true);
}
     
void loop(void) {
  // update finger positions
  updateFingerVals();
  
  // compute orientation
  updateOrientation();

  // sensor variables
  sensors_event_t angVelocityData , linearAccelData;
  
  // get gyro and accelerometer data
  bnoWrist.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bnoWrist.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // convert angular velocity data
  angularVelocity.x = angVelocityData.gyro.x;
  angularVelocity.y = angVelocityData.gyro.y;
  angularVelocity.z = angVelocityData.gyro.z;

  // convert linear acceleration data
  linearAcceleration.x = linearAccelData.acceleration.x;
  linearAcceleration.y = linearAccelData.acceleration.y;
  linearAcceleration.z = linearAccelData.acceleration.z;
  
  // publish imu data for dead reckoning
  orientation_pub.publish(&orientation);   
  angularVel_pub.publish(&angularVelocity);   
  linearAccel_pub.publish(&linearAcceleration);

  // publish finger values
  fingerVals_pub.publish(&finger_vals);
  
  nh.spinOnce();
  delay(10);
}

void updateOrientation() {
  imu::Quaternion quat = bnoWrist.getQuat();
  orientation.x = quat.x();
  orientation.y = quat.y();
  orientation.z = quat.z();
  orientation.w = quat.w();
}

void updateFingerVals() {
  int fingers[5];

  // read haptive glove finger positions
  fingers[0] = analogRead(A2);
  fingers[1] = analogRead(A1);
  fingers[2] = analogRead(A0);

  // store values in ros multiarray
  finger_vals.data[0] = fingers[0];
  finger_vals.data[1] = fingers[1];
  finger_vals.data[2] = fingers[2];
  finger_vals.data[3] = fingers[2];
  finger_vals.data[4] = fingers[2];
}

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
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

//imu
Adafruit_BNO055 bnoWrist = Adafruit_BNO055(55);

// global variables to pusblish
geometry_msgs::QuaternionStamped orientation;
geometry_msgs::Vector3Stamped linearAcceleration;
geometry_msgs::Vector3Stamped angularVelocity;
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
  nh.loginfo("let's go!!");
  delay(1000);
  
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
  angularVelocity.vector.x = angVelocityData.gyro.x;
  angularVelocity.vector.y = angVelocityData.gyro.y;
  angularVelocity.vector.z = angVelocityData.gyro.z;
  angularVelocity.header.stamp = nh.now();

  // convert linear acceleration data
  linearAcceleration.vector.x = linearAccelData.acceleration.x;
  linearAcceleration.vector.y = linearAccelData.acceleration.y;
  linearAcceleration.vector.z = linearAccelData.acceleration.z;
  linearAcceleration.header.stamp = nh.now();
  
  // publish imu data for dead reckoning
  orientation_pub.publish(&orientation);   
  angularVel_pub.publish(&angularVelocity);   
  linearAccel_pub.publish(&linearAcceleration);

  // publish finger values
  //fingerVals_pub.publish(&finger_vals);
  
  nh.spinOnce();
  delay(10);
}

void updateOrientation() {
  imu::Quaternion quat = bnoWrist.getQuat();
  orientation.quaternion.x = quat.x();
  orientation.quaternion.y = quat.y();
  orientation.quaternion.z = quat.z();
  orientation.quaternion.w = quat.w();
  orientation.header.stamp = nh.now();
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

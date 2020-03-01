//#include "SCoop.h"
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
sensor_msgs::Range range_msg2;
sensor_msgs::Range range_msg3;
sensor_msgs::Range range_msg4;
sensor_msgs::Range range_msg5;
sensor_msgs::Range range_msg6;
ros::Publisher pub_range("/sonar1", &range_msg);
ros::Publisher pub_range2("/sonar2", &range_msg2);
ros::Publisher pub_range3("/sonar3", &range_msg3);
ros::Publisher pub_range4("/sonar4", &range_msg4);
ros::Publisher pub_range5("/sonar5", &range_msg5);
ros::Publisher pub_range6("/sonar6", &range_msg6);

/*unsigned long high_level_time() {
  
}*/

float getRange_Ultrasound(int trig_pin, int echo_pin) {
  float distance = 0;
  float time_echo_us = 0;
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(50);
  digitalWrite(trig_pin, LOW);

  // add by: dengke
  // while timeout=13500, max_range=30cm，fre=10HZ
  // need to test and select a set of optimal max_range and fre
  time_echo_us = pulseIn(echo_pin, HIGH, 13500);
  if(time_echo_us < 1764 && time_echo_us > 100) {
    distance = time_echo_us*34/100000/2;
  }
  else {
    distance = 10000;
  }
  return distance;
}

void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(pub_range);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);
  nh.advertise(pub_range5);
  nh.advertise(pub_range6);
  int trig_pin = 0;
  int echo_pin = 0;
  for(trig_pin=30; trig_pin<=44; trig_pin++) {
    pinMode(trig_pin, OUTPUT);
    trig_pin++;
  }
  for(echo_pin=31; echo_pin<=45; echo_pin++) {
    pinMode(echo_pin, INPUT);
    echo_pin++;
  }
}

void loop() {
  range_msg.header.stamp = nh.now();
  range_msg.header.frame_id = "sonar1";
  range_msg.range = getRange_Ultrasound(30, 31);
  range_msg2.header.stamp = nh.now();
  range_msg2.header.frame_id = "sonar2";
  range_msg2.range = getRange_Ultrasound(32, 33);
  range_msg3.header.stamp = nh.now();
  range_msg3.header.frame_id = "sonar3";
  range_msg3.range = getRange_Ultrasound(34, 35);
  range_msg4.header.stamp = nh.now();
  range_msg4.header.frame_id = "sonar4";
  range_msg4.range = getRange_Ultrasound(36, 37);
  range_msg5.header.stamp = nh.now();
  range_msg5.header.frame_id = "sonar5";
  range_msg5.range = getRange_Ultrasound(38, 39);
  range_msg6.header.stamp = nh.now();
  range_msg6.header.frame_id = "sonar6";
  range_msg6.range = getRange_Ultrasound(40, 41);
  pub_range.publish(&range_msg);
  pub_range2.publish(&range_msg2);
  pub_range3.publish(&range_msg3);
  pub_range4.publish(&range_msg4);
  pub_range5.publish(&range_msg5);
  pub_range6.publish(&range_msg6);
  nh.spinOnce();
}

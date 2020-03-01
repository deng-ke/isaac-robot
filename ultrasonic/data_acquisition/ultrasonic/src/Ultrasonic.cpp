/*
Copyright: raydiculous
Author: Deng Ke
Date: 2019-09-20
Description: collect ultrasonic sensor data and publish it
*/

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include <sstream>
#include <fcntl.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <string>
#include <fstream>
#define SYSFS_GPIO_DIR "/sys/class/gpio/"
#define MAX_BUF 64

using namespace std;

class PubSonarData {
  public:
    PubSonarData(const char* trig_pin, const char* echo_pin) {
      pub_ = n_.advertise<sensor_msgs::Range>("Ultrasonic", 1);
      strcpy(this->trig_pin_, trig_pin);
      strcpy(this->echo_pin_, echo_pin);
    }
    ~PubSonarData() {
      resetGpioDir();
      strcat(gpio_dir_,"unexport");
      data_stream_.open(gpio_dir_, std::ios::app | std::ios::out);
      data_stream_ << trig_pin_;
      data_stream_.close();
      data_stream_.open(gpio_dir_, std::ios::app | std::ios::out);
      data_stream_ << echo_pin_;
      data_stream_.close();
    }
    void resetGpioDir();
    void setGpio();
    void readData();
    void pubData();
  private:
    char gpio_dir_[30] = "/sys/class/gpio/";
    double distance_ = 0;
    char trig_pin_[3];
    char echo_pin_[3];
    fstream data_stream_;
    sensor_msgs::Range sonar_msg;
    ros::NodeHandle n_;
    ros::Publisher pub_;
};

void PubSonarData::resetGpioDir() {
  strcpy(gpio_dir_,"/sys/class/gpio/");
}

void PubSonarData::setGpio() {
  // export trig_pin_
  resetGpioDir();
  data_stream_.open(strcat(gpio_dir_,"export"), std::ios::app | std::ios::out);
  data_stream_ << trig_pin_;
  data_stream_.close();
  sleep(1);
  // set direction to out
  resetGpioDir();
  strcat(gpio_dir_, "gpio");strcat(gpio_dir_, trig_pin_);strcat(gpio_dir_, "/direction");
  data_stream_.open(gpio_dir_, std::ios::app | std::ios::out);
  data_stream_ << "out";
  data_stream_.close();

  // export echo_pin_
  resetGpioDir();
  data_stream_.open(strcat(gpio_dir_,"export"), std::ios::app | std::ios::out);
  data_stream_ << echo_pin_;
  data_stream_.close();
  sleep(1);
  // set direction to in
  resetGpioDir();
  strcat(gpio_dir_, "gpio");strcat(gpio_dir_, echo_pin_);strcat(gpio_dir_, "/direction");
  data_stream_.open(gpio_dir_, std::ios::app | std::ios::out);
  data_stream_ << "in";
  data_stream_.close();
}

void PubSonarData::readData() {
  
  // give high-level to trig_pin
  resetGpioDir();
  strcat(gpio_dir_, "gpio");strcat(gpio_dir_, trig_pin_);strcat(gpio_dir_, "/value");
  data_stream_.open(gpio_dir_, std::ios::app | std::ios::out);
  data_stream_ << "1";
  data_stream_.close();
  usleep(50);
  data_stream_.open(gpio_dir_, std::ios::app | std::ios::out);
  data_stream_ << "0";
  data_stream_.close();

  // read data from echo_pin
  FILE *stream;
  char data[10];
  clock_t start,end;
  resetGpioDir();
  strcat(gpio_dir_, "gpio");strcat(gpio_dir_, echo_pin_);strcat(gpio_dir_, "/value");
  stream = fopen(gpio_dir_,"r+");
  fscanf(stream,"%s",data);
  fclose(stream);
  if(data[0] == '0') {
    while(1) {
      stream = fopen(gpio_dir_,"r+");
      fscanf(stream,"%s",data);
      fclose(stream);
      if(data[0] == '1') {
        start = clock();
        break;
      }
    }
    while(1) {
      stream = fopen(gpio_dir_,"r+");
      fscanf(stream,"%s",data);
      fclose(stream);
      if(data[0] == '0') {
        end = clock();
        break;
      }
    }
    double time = (double)(end - start) / CLOCKS_PER_SEC;
    distance_ = time*340/2;
  }
  else {
    std::cout << "ERROR" << std::endl;
  }
}

void PubSonarData::pubData() {
  sonar_msg.header.stamp = ros::Time::now();
  sonar_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg.min_range = 0;
  sonar_msg.max_range = 1;
  sonar_msg.range = distance_;
  pub_.publish(sonar_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ultrasonic");
  PubSonarData sonar("44", "45");
  sonar.setGpio();
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    std::cout << "111" << std::endl;
    sonar.readData();
    sonar.pubData();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

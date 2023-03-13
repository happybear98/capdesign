/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <unistd.h>

/*
#define HAVE_ROUND
#ifdef _DEBUG
#define RESTORE_DEBUG
#undef _DEBUG
#endif
#include <Python.h>
#ifdef RESTORE_DEBUG
#define _DEBUG
#undef RESTORE_DEBUG
#endif
*/

//motor include start
#include "wiringPi.h"
#include <softPwm.h>
//motor include end

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#include <time.h>
#include <fstream>
#include <csignal>
#include <stdlib.h>

#define ROS2Verision "1.0.1"

//#define RAD2DEG(x) ((x)*180./M_PI)


using namespace std;

//motor value start
// FL-MOTER
#define EN_FL 23
#define IN_FL_1 22
#define IN_FL_2 21
int OUT_FL_1 = LOW;
int OUT_FL_2 = LOW;

// BL-MOTER
#define EN_BL 29
#define IN_BL_1 28
#define IN_BL_2 27
int OUT_BL_1 = LOW;
int OUT_BL_2 = LOW;

// FR-MOTER
#define EN_FR 4
#define IN_FR_1 5
#define IN_FR_2 6
int OUT_FR_1 = LOW;
int OUT_FR_2 = LOW;

// BR-MOTER
#define EN_BR 3
#define IN_BR_1 2
#define IN_BR_2 0
int OUT_BR_1 = LOW;
int OUT_BR_2 = LOW;

string st_ready = "ready";
string st_go = "go";
string st_back = "back";
string st_right = "right";
string st_left = "left";

 void state_value(string state) {
    if (state == "ready") {
      OUT_FL_1 = OUT_FL_2 = OUT_FR_1 = OUT_FR_2 = OUT_BL_1 =  OUT_BL_2 = OUT_BR_1 = OUT_BR_2 = LOW;
      softPwmWrite(EN_FL, 0);
      softPwmWrite(EN_BL, 0);
      softPwmWrite(EN_FR, 0);
      softPwmWrite(EN_BR, 0);
    }
    else if (state == "go") {
      OUT_FL_1 = OUT_FR_2 = OUT_BL_1 = OUT_BR_2 = LOW;
      OUT_FL_2 = OUT_FR_1 = OUT_BL_2 = OUT_BR_1 = HIGH;
      softPwmWrite(EN_FL, 100);
      softPwmWrite(EN_BL, 100);
      softPwmWrite(EN_FR, 100);
      softPwmWrite(EN_BR, 100);
    }
    else if (state == "back") {
      OUT_FL_1 = OUT_FR_2 = OUT_BL_1 = OUT_BR_2 = HIGH;
      OUT_FL_2 = OUT_FR_1 = OUT_BL_2 = OUT_BR_1 = LOW;
      softPwmWrite(EN_FL, 100);
      softPwmWrite(EN_BL, 100);
      softPwmWrite(EN_FR, 100);
      softPwmWrite(EN_BR, 100);
    }
    else if (state == "right") {
      OUT_FL_1 = OUT_FR_1 = OUT_BL_1 = OUT_BR_1 = HIGH;
      OUT_FL_2 = OUT_FR_2 = OUT_BL_2 = OUT_BR_2 = LOW;
      softPwmWrite(EN_FL, 100);
      softPwmWrite(EN_BL, 100);
      softPwmWrite(EN_FR, 100);
      softPwmWrite(EN_BR, 100);
    }
    else if (state == "left") {
      OUT_FL_2 = OUT_FR_2 = OUT_BL_2 = OUT_BR_2 = HIGH;
      OUT_FL_1 = OUT_FR_1 = OUT_BL_1 = OUT_BR_1 = LOW;
      softPwmWrite(EN_FL, 100);
      softPwmWrite(EN_BL, 100);
      softPwmWrite(EN_FR, 100);
      softPwmWrite(EN_BR, 100);
    }
  }
//motor value end



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  wiringPiSetup();
	
   //motor code start
  // FL-MOTER
  pinMode(EN_FL, OUTPUT);
  pinMode(IN_FL_1, OUTPUT);
  pinMode(IN_FL_2, OUTPUT);
  softPwmCreate(EN_FL, 0, 100);
  softPwmWrite(EN_FL, 0);

  // BL-MOTER
  pinMode(EN_BL, OUTPUT);
  pinMode(IN_BL_1, OUTPUT);
  pinMode(IN_BL_2, OUTPUT);
  softPwmCreate(EN_BL, 0, 100);
  softPwmWrite(EN_BL, 0);

  // FR-MOTER
  pinMode(EN_FR, OUTPUT);
  pinMode(IN_FR_1, OUTPUT);
  pinMode(IN_FR_2, OUTPUT);
  softPwmCreate(EN_FR, 0, 100);
  softPwmWrite(EN_FR, 0);

  // BR-MOTER
  pinMode(EN_BR, OUTPUT);
  pinMode(IN_BR_1, OUTPUT);
  pinMode(IN_BR_2, OUTPUT);
  softPwmCreate(EN_BR, 0, 100);
  softPwmWrite(EN_BR, 0);

  string input;
  //motor code end


  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

  CYdLidar laser;
  std::string str_optvalue = "/dev/ydlidar";
  node->declare_parameter("port");
  node->get_parameter("port", str_optvalue);
  ///lidar port
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  ///ignore array
  str_optvalue = "";
  node->declare_parameter("ignore_array");
  node->get_parameter("ignore_array", str_optvalue);
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  std::string frame_id = "laser_frame";
  node->declare_parameter("frame_id");
  node->get_parameter("frame_id", frame_id);

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 230400;
  node->declare_parameter("baudrate");
  node->get_parameter("baudrate", optval);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TRIANGLE;
  node->declare_parameter("lidar_type");
  node->get_parameter("lidar_type", optval);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  node->declare_parameter("device_type");
  node->get_parameter("device_type", optval);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 5;
  node->declare_parameter("sample_rate");
  node->get_parameter("sample_rate", optval);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  node->declare_parameter("abnormal_check_count");
  node->get_parameter("abnormal_check_count", optval);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
     

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  node->declare_parameter("fixed_resolution");
  node->get_parameter("fixed_resolution", b_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  b_optvalue = true;
  node->declare_parameter("reversion");
  node->get_parameter("reversion", b_optvalue);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  b_optvalue = true;
  node->declare_parameter("inverted");
  node->get_parameter("inverted", b_optvalue);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  node->declare_parameter("auto_reconnect");
  node->get_parameter("auto_reconnect", b_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
  node->declare_parameter("isSingleChannel");
  node->get_parameter("isSingleChannel", b_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  node->declare_parameter("intensity");
  node->get_parameter("intensity", b_optvalue);
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = false;
  node->declare_parameter("support_motor_dtr");
  node->get_parameter("support_motor_dtr", b_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  node->declare_parameter("angle_max");
  node->get_parameter("angle_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  node->declare_parameter("angle_min");
  node->get_parameter("angle_min", f_optvalue);
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 16.f;
  node->declare_parameter("range_max");
  node->get_parameter("range_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.1f;
  node->declare_parameter("range_min");
  node->get_parameter("range_min", f_optvalue);
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 5.f;
  node->declare_parameter("frequency");
  node->get_parameter("frequency", f_optvalue);
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  bool invalid_range_is_inf = false;
  node->declare_parameter("invalid_range_is_inf");
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);


  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }
  
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  auto stop_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan",stop_scan_service);

  auto start_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan",start_scan_service);

  rclcpp::WallRate loop_rate(20);


  while (ret && rclcpp::ok()) {

    LaserScan scan;//

    if (laser.doProcessSimple(scan)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;
      


    float range_data[1050] = {0};

      int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);

      for(size_t i=0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
        if(index >=0 && index < size) {
          scan_msg->ranges[index] = scan.points[i].range;
          scan_msg->intensities[index] = scan.points[i].intensity;

	    range_data[i] = scan.points[i].range;
	}
      }

//my code start
	//data input code start
	    const int num_arrays = 360;
	    int points = scan.points.size();

	    int arr_size = sizeof(range_data) / sizeof(range_data[0]);
	    std::vector<float> data(range_data, range_data + arr_size);
	    vector<vector<float>> arrays(num_arrays);

	    //int elements_per_array = points / num_arrays;

		int index = 0;

		for (int j = 0; j < num_arrays; j++) {
		    int num_elements = points / num_arrays;
		    if(j < points % num_arrays) {
			num_elements++;
		    }
		    for(int k = 0; k < num_elements; k++) {
			arrays[j].push_back(data[index]);
			index++;
		    }

	           // for (int k = j * elements_per_array; k < (j + 1) * elements_per_array; k++) {
		   //     arrays[j].push_back(data[k]);
	           // }
	        } //data input code end



	//close warning code start
	    float arr_sum = 0;
	    float col_sum = 0;

	    for (int a = 0; a < 360; a++) {
		for (long unsigned int b = 0; b < arrays[a].size(); b++) {
		    if (arrays[a][b] != 0) {

			if ((a < 90) && (arrays[a][b] < 0.65)) {
			    cout << "\n\n\nFRONT WARNING!\n\n\n" << endl;
			    state_value(st_ready);
                            //sleep(0.3);
                            digitalWrite(IN_FL_1, OUT_FL_1);
                            digitalWrite(IN_FL_2, OUT_FL_2);
                            digitalWrite(IN_BL_1, OUT_BL_1);
                            digitalWrite(IN_BL_2, OUT_BL_2);

                            digitalWrite(IN_FR_1, OUT_FR_1);
                            digitalWrite(IN_FR_2, OUT_FR_2);
                            digitalWrite(IN_BR_1, OUT_BR_1);
                            digitalWrite(IN_BR_2, OUT_BR_2);
                            sleep(1);
			}
			else if ((90 <= a) && (a < 180) && (arrays[a][b] < 0.5)) {
			    cout << "\n\n\nRIGNT WARNING!\n\n\n" << endl;
			}
			else if ((180 <= a) && (a < 270) && (arrays[a][b] < 0.65)) {
			    cout << "\n\n\nBACK WARNGING!\n\n\n" << endl;
			    state_value(st_go);
                            //sleep(0.3);
                            digitalWrite(IN_FL_1, OUT_FL_1);
                            digitalWrite(IN_FL_2, OUT_FL_2);
                            digitalWrite(IN_BL_1, OUT_BL_1);
                            digitalWrite(IN_BL_2, OUT_BL_2);

                            digitalWrite(IN_FR_1, OUT_FR_1);
                            digitalWrite(IN_FR_2, OUT_FR_2);
                            digitalWrite(IN_BR_1, OUT_BR_1);
                            digitalWrite(IN_BR_2, OUT_BR_2);
                            sleep(1);
			}
			else if ((270 <= a) && (a < 360) && (arrays[a][b] < 0.5)) {
			    cout << "\n\n\nLEFT WARNGING!\n\n\n" << endl;
			}
			else {
			    state_value(st_ready);
			    //sleep(0.3);
			    digitalWrite(IN_FL_1, OUT_FL_1);
                            digitalWrite(IN_FL_2, OUT_FL_2);
                            digitalWrite(IN_BL_1, OUT_BL_1);
                            digitalWrite(IN_BL_2, OUT_BL_2);

                            digitalWrite(IN_FR_1, OUT_FR_1);
                            digitalWrite(IN_FR_2, OUT_FR_2);
                            digitalWrite(IN_BR_1, OUT_BR_1);
                            digitalWrite(IN_BR_2, OUT_BR_2);
                            sleep(1);
			}
		    }
        //usleep(500);
		}

		//else cout << "\n\n\n-\n\n\n" << endl;

	    }



	    //if (avg < 0.7) { cout<<"\n\n\n\t\tWARNING TOO CLOSE\n\n\n"<<endl; }
	    //else { cout<<"\n\n\n\t\tIT's OK\n\n\n"<<endl; }
	//close warning code end



	//array output code start
	/*for(int a = 0; a < num_arrays; a++) {
	    cout << "Array " << a << ": ";
	    for(long unsigned int b = 0; b < arrays[a].size(); b++) {
		cout << arrays[a][b] << " ";
	    }
	    cout << endl;
	}*/ //array output code end
//my code end

      laser_pub->publish(*scan_msg);

    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    if(!rclcpp::ok()) {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();
  //Py_Finalize();

  return 0;
}


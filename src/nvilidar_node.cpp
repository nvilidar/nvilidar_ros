#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "CNviLidar.h"
#include <vector>
#include <iostream>
#include <string.h>
#include "mysignal.h"

using namespace nvilidar;

#define ROSVerision "1.0.1"


std::vector<float> split(const std::string &s, char delim) 
{
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "nvilidar_node"); 
    printf(" _   ___      _______ _      _____ _____          _____ \n");
    printf("| \\ | \\ \\    / /_   _| |    |_   _|  __ \\   /\\   |  __ \\\n");
    printf("|  \\| |\\ \\  / /  | | | |      | | | |  | | /  \\  | |__) |\n");
    printf("| . ` | \\ \\/ /   | | | |      | | | |  | |/ /\\ \\ |  _  / \n");
    printf("| |\\  |  \\  /   _| |_| |____ _| |_| |__| / ____ \\| | \\ \\\n");
    printf("|_| \\_|   \\/   |_____|______|_____|_____/_/    \\_\\_|  \\ \\\n");
    printf("\n");
    fflush(stdout);

    //sig init 
    nvilidar::SigInit();
  
    std::string port;
    int baudrate = 921600;
    std::string frame_id;
    bool reversion, resolution_fixed;
    bool auto_reconnect;
    double angle_max,angle_min;
    result_t op_result;
    std::string list;
    std::vector<float> ignore_array;  
    double max_range, min_range;
    double frequency;
    int samp_rate = 20;
    bool inverted = false;
    bool isSingleChannel = false;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/nvilidar"); 
    nh_private.param<int>("baudrate", baudrate, 921600); 
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("resolution_fixed", resolution_fixed, "true");
    nh_private.param<bool>("auto_reconnect", auto_reconnect, "true");
    nh_private.param<bool>("reversion", reversion, "false");
    nh_private.param<double>("angle_max", angle_max , 180);
    nh_private.param<double>("angle_min", angle_min , -180);
    nh_private.param<double>("range_max", max_range , 64.0);
    nh_private.param<double>("range_min", min_range , 0.01);
    nh_private.param<double>("frequency", frequency , 10.0);
    nh_private.param<std::string>("ignore_array",list,"");
    nh_private.param<int>("samp_rate", (samp_rate), (samp_rate));
    nh_private.param<bool>("isSingleChannel", isSingleChannel, isSingleChannel);
 

    ignore_array = split(list ,',');
    if(ignore_array.size()%2){
        ROS_ERROR_STREAM("ignore array is odd need be even");
    }

    for(uint16_t i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            ROS_ERROR_STREAM("ignore array should be between 0 and 360");
        }
    }

    CNviLidar laser;

    if(angle_max < angle_min){
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }

    ROS_INFO("[NVILIDAR INFO] Now NVILIDAR ROS SDK VERSION:%s .......", ROSVerision);
    laser.setSerialPort(port);
    laser.setSerialBaudrate(baudrate);
    laser.setMaxRange(max_range);
    laser.setMinRange(min_range);
    laser.setMaxAngle(angle_max);
    laser.setMinAngle(angle_min);
    laser.setReversion(reversion);
    laser.setFixedResolution(resolution_fixed);
    laser.setAutoReconnect(auto_reconnect);
    laser.setScanFrequency(frequency);
    laser.setIgnoreArray(ignore_array);
    laser.setInverted(inverted);
    laser.setSingleChannel(isSingleChannel);
    bool ret = laser.LidarInitialize();
    if (ret) {
        ret = laser.LidarTurnOn();
        if (!ret) {
            ROS_ERROR("Failed to start scan mode!!!");
        }
    } else {
        ROS_ERROR("Error initializing NVILIDAR Comms and Status!!!");
    }
    ros::Rate rate(50);

    while (ret && ros::ok && nvilidar::isOK()) 
    {
       // bool hardError;
         LidarScan scan;
         if(laser.LidarSamplingProcess(scan ))
         {
            sensor_msgs::LaserScan scan_msg;
            ros::Time start_scan_time;
            start_scan_time.sec = scan.stamp/1000000000ul;
            start_scan_time.nsec = scan.stamp%1000000000ul;
            scan_msg.header.stamp = start_scan_time;
            scan_msg.header.frame_id = frame_id;
            scan_msg.angle_min =(scan.config.min_angle);
            scan_msg.angle_max = (scan.config.max_angle);
            scan_msg.angle_increment = (scan.config.angle_increment);
            scan_msg.scan_time = scan.config.scan_time;
            scan_msg.time_increment = scan.config.time_increment;
            scan_msg.range_min = (scan.config.min_range);
            scan_msg.range_max = (scan.config.max_range);
            int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
            scan_msg.ranges.resize(size);
            scan_msg.intensities.resize(size);
            for(int i=0; i < scan.points.size(); i++) {
                int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
                if(index >=0 && index < size) 
                {
                     scan_msg.ranges[index] = scan.points[i].range;
                     scan_msg.intensities[index] = scan.points[i].intensity;
                }
            }
           scan_pub.publish(scan_msg);
        }  
        rate.sleep();
        ros::spinOnce();
    }

    laser.LidarTurnOff();
    ROS_INFO("[NVILIDAR INFO] Now NVILIDAR is stopping .......");
    laser.LidarCloseHandle();
    return 0;
}

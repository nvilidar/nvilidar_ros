#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <string.h>
#include "sensor_msgs/LaserScan.h"
#include "nvilidar_driver_serialport.h"
#include "nvilidar_dataprocess.h"
#include "mysignal.h"
using namespace nvilidar;

#define ROSVerision "1.0.2"

int main(int argc, char * argv[]) 
{
    ros::init(argc, argv, "nvilidar_node"); 
    printf(" _   ___      _______ _      _____ _____          _____ \n");
    printf("| \\ | \\ \\    / /_   _| |    |_   _|  __ \\   /\\   |  __ \\\n");
    printf("|  \\| |\\ \\  / /  | | | |      | | | |  | | /  \\  | |__) |\n");
    printf("| . ` | \\ \\/ /   | | | |      | | | |  | |/ /\\ \\ |  _  / \n");
    printf("| |\\  |  \\  /   _| |_| |____ _| |_| |__| / ____ \\| | \\ \\\n");
    printf("|_| \\_|   \\/   |_____|______|_____|_____/_/    \\_\\_|  \\ \\\n");
    printf("\n");
    fflush(stdout);

    //参数配置 
    Nvilidar_UserConfigTypeDef  cfg;
    Nvilidar_UserConfigTypeDef  cfg_default;
    //获取默认参数  如需要修改 可以进行修改  
	LidarDefaultUserConfig(cfg_default);

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");

    //读取雷达配置 从rviz文件内 如果里面有配置对应参数  则走里面的数据。如果没有，则直接取“nvilidar_dataprocess.h”里面的默认参数配置信息
    nh_private.param<std::string>("serialport_name", cfg.serialport_name, cfg_default.serialport_name); 
    nh_private.param<int>("serialport_baud", cfg.serialport_baud, cfg_default.serialport_baud); 
    nh_private.param<std::string>("frame_id", cfg.frame_id, cfg_default.frame_id);
    nh_private.param<bool>("resolution_fixed", cfg.resolution_fixed, cfg_default.resolution_fixed);
    nh_private.param<bool>("auto_reconnect", cfg.auto_reconnect, cfg_default.auto_reconnect);
    nh_private.param<bool>("reversion", cfg.reversion, cfg_default.reversion);
    nh_private.param<bool>("inverted", cfg.inverted, cfg_default.inverted);
    nh_private.param<double>("angle_max", cfg.angle_max , cfg_default.angle_max);
    nh_private.param<double>("angle_min", cfg.angle_min , cfg_default.angle_min);
    nh_private.param<double>("range_max", cfg.range_max , cfg_default.range_max);
    nh_private.param<double>("range_min", cfg.range_min , cfg_default.range_min);
    nh_private.param<double>("aim_speed", cfg.aim_speed , cfg_default.aim_speed);
    nh_private.param<int>("sampling_rate", cfg.sampling_rate, cfg_default.sampling_rate);
    nh_private.param<bool>("sensitive",      cfg.sensitive, cfg_default.sensitive);
    nh_private.param<int>("tailing_level",  cfg.tailing_level, cfg_default.tailing_level);
    nh_private.param<double>("angle_offset",  cfg.angle_offset, cfg_default.angle_offset);
    nh_private.param<bool>("single_channel",  cfg.single_channel, cfg_default.single_channel);
    nh_private.param<std::string>("ignore_array_string",  cfg.ignore_array_string, cfg_default.ignore_array_string);

    //同步数据至雷达参数列表  
    LidarParaSync(cfg);

    
	CircleDataInfoTypeDef node_circle;
	Nvilidar_DeviceInfo info;
	static uint32_t  no_response_times = 0;

    nvilidar::LidarDriverSerialport laser(cfg);

    ROS_INFO("[NVILIDAR INFO] Now NVILIDAR ROS SDK VERSION:%s .......", ROSVerision);

    //初始化 变量定义 
    bool ret = laser.LidarInit();
    if (ret) 
    {
        //启动雷达 
        ret = laser.StartScan();
        if (!ret) 
        {
            ROS_ERROR("Failed to start Scan!!!");
        }
    } 
    else 
    {
        ROS_ERROR("Error initializing NVILIDAR Comms and Status!!!");
    }
    ros::Rate rate(50);

    while (ret && ros::ok()) 
    {
         LidarScan scan;
         if(laser.waitCircleResponse(node_circle))
         {
             //点集格式转换 
			LidarSamplingData(cfg, node_circle, scan);

            sensor_msgs::LaserScan scan_msg;
            ros::Time start_scan_time;
            start_scan_time.sec = scan.stamp/1000000000ul;
            start_scan_time.nsec = scan.stamp%1000000000ul;
            scan_msg.header.stamp = start_scan_time;
            scan_msg.header.frame_id = "laser_frame";
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
    laser.LidarStopScan();
    ROS_INFO("[NVILIDAR INFO] Now NVILIDAR is stopping .......");
    laser.LidarCloseHandle();
    return 0;
}

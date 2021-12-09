#pragma once

#include "nvilidar_dataprocess.h"
#include "nvilidar_def.h"
#include "nvilidar_protocol.h"
#include <cmath>
#include <vector>
#include "myconsole.h"
#include <iostream> 
#include <istream> 
#include <sstream>

//pi

#ifndef M_PI
	#define M_PI        3.14159265358979323846
#endif 

/**
 * @brief The Laser Point struct
 * @note angle unit: rad.\n
 * range unit: meter.\n
 */
typedef struct {
	/// lidar angle. unit(rad)
	float angle;
	/// lidar range. unit(m)
	float range;
	/// lidar intensity
	float intensity;
} NviLidarPoint;

/**
 * @brief A struct for returning configuration from the NVILIDAR
 * @note angle unit: rad.\n
 * time unit: second.\n
 * range unit: meter.\n
 */
typedef struct {
	/// Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing NVILIDAR from the top.
	float min_angle;
	/// Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing NVILIDAR from the top.
	float max_angle;
	/// angle resoltuion [rad]
	float angle_increment;
	/// Scan resoltuion [s]
	float time_increment;
	/// Time between scans
	float scan_time;
	/// Minimum range [m]
	float min_range;
	/// Maximum range [m]
	float max_range;
} NviLidarConfig;


typedef struct {
	/// System time when first range was measured in nanoseconds
	uint64_t stamp;
	/// Array of lidar points
	std::vector<NviLidarPoint> points;
	/// Configuration of scan
	NviLidarConfig config;
} LidarScan;

//拆分自行实现  
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


//同步数据信息  相关信息 同步到雷达 
inline void LidarParaSync(Nvilidar_UserConfigTypeDef &cfg)
{
	cfg.storePara.samplingRate = (uint32_t)(cfg.sampling_rate *1000);		// * 1000
	cfg.storePara.angleOffset = (uint16_t)(cfg.angle_offset * 64 + 0.5);	//角度偏移 	实际与雷达的  64倍 U16 	
	cfg.storePara.isHasSensitive = cfg.sensitive;							//是否带有信号质量 
	cfg.storePara.aimSpeed = (uint16_t)(cfg.aim_speed *100 + 0.5);			//N Hz 实际与雷达的  100倍 U16 
	cfg.storePara.tailingLevel = cfg.tailing_level;							//拖尾等级 

	//ingnore array拆分 
	cfg.ignore_array = split(cfg.ignore_array_string ,',');
    if(cfg.ignore_array.size()%2)
	{
        nvilidar::console.error("ignore array is odd need be even");
    }
    for(uint16_t i =0 ; i < cfg.ignore_array.size();i++)
	{
        if(cfg.ignore_array[i] < -180.0 && cfg.ignore_array[i] > 180.0)
		{
            nvilidar::console.error("ignore array should be between 0 and 360");
        }
    }
}

//初始参数 
inline void LidarDefaultUserConfig(Nvilidar_UserConfigTypeDef &cfg)
{
	//配置参数 
	cfg.serialport_baud = 921600;
	cfg.serialport_name = "/dev/nvilidar";
	cfg.frame_id = "laser_frame";
	cfg.resolution_fixed = false;		//非固定角分辨 
	cfg.auto_reconnect = false;			//自动重连 
	cfg.reversion = false;				//倒转 
	cfg.inverted = false;				//180度 
	cfg.angle_max = 180.0;
	cfg.angle_min = -180.0;
	cfg.range_max = 64.0;
	cfg.range_min = 0;
	cfg.aim_speed = 10.0;				//10Hz
	cfg.sampling_rate = 10;				//10k
	cfg.sensitive = false;				//数据不加信号质量 
	cfg.tailing_level = false;			//拖尾等级  
	cfg.angle_offset = 0.0;				//角度偏移 
	cfg.single_channel = false;			//单通道 
	cfg.ignore_array_string = "";				//过滤部分角度信息 

	LidarParaSync(cfg);
}

//采样数据分析  
inline void LidarSamplingData(Nvilidar_UserConfigTypeDef cfg, CircleDataInfoTypeDef info, LidarScan &outscan)
{
	uint32_t all_nodes_counts = 0;		//所有点数  不做截取等用法 
	uint64_t scan_time = 0;				//2圈点的扫描间隔 

	//扫描时间 
	scan_time = info.stopStamp - info.startStamp;

	//清空接收数据  
	outscan.points.clear();

	//原始数据  计数
	uint32_t lidar_ori_count = info.lidarCircleNodePoints.size();		

	//固定角分辨率 
	if(cfg.resolution_fixed)
	{
		all_nodes_counts = (uint32_t)(cfg.storePara.samplingRate *100 / cfg.storePara.aimSpeed);
	}
	else 	//非固定角分辨率 则是雷达默认一包多少点 就实际产生多少个点 
	{
		all_nodes_counts = lidar_ori_count;
	}

	//最大角与最小角问题 如是不对  则反转  
	if (cfg.angle_max < cfg.angle_min)
	{
		float temp = cfg.angle_min;
		cfg.angle_min = cfg.angle_max;
		cfg.angle_max = temp;
	}

	//以角度为比例  计算真实的点数信息 
	int output_count = all_nodes_counts * ((cfg.angle_max - cfg.angle_min) / 360.0f);   

	outscan.stamp = info.startStamp;                
	outscan.config.max_angle = cfg.angle_max*M_PI / 180.0;			//计算最大角度  				
	outscan.config.min_angle = cfg.angle_min*M_PI / 180.0;			//计算最小角度  
	outscan.config.angle_increment = (outscan.config.max_angle -	//计算2点之间的角度增量 		
		outscan.config.min_angle) /
		(double)(output_count - 1);
	outscan.config.scan_time = static_cast<float>(1.0 * scan_time / 1e9);  	//扫描时间信息  
	outscan.config.time_increment = outscan.config.scan_time / (double)(all_nodes_counts - 1); 	//2点之间的时间 
	outscan.config.min_range = cfg.range_min;          
	outscan.config.max_range = cfg.range_max;         

	//初始化变量  
	float dist = 0.0;
	float angle = 0.0;
	float intensity = 0.0;
	unsigned int i = 0;

	//从雷达原始数据中  提取数据  
	for (; i < lidar_ori_count; i++)
	{
		dist = static_cast<float>(info.lidarCircleNodePoints.at(i).lidar_distance / 1000.f);
		intensity = static_cast<float>(info.lidarCircleNodePoints.at(i).lidar_quality);
		angle = static_cast<float>(info.lidarCircleNodePoints.at(i).lidar_angle);
		angle = angle * M_PI / 180.0;
		angle = 2 * M_PI - angle;

		//Rotate 180 degrees or not
		if (cfg.reversion)
		{
			angle = angle + M_PI;
		}
		//Is it counter clockwise
		if (cfg.inverted)
		{
			angle = 2 * M_PI - angle;
		}

		//-pi ~ pi
		angle = fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
		if (angle > M_PI)
		{
			angle -= 2.0 * M_PI;
		}

		//忽略点（事先配置好哪个角度的范围）
		if (cfg.ignore_array.size() != 0)
		{
			for (uint16_t j = 0; j < cfg.ignore_array.size(); j = j + 2)
			{
				double angle_start = cfg.ignore_array[j] * M_PI / 180.0;
				double angle_end = cfg.ignore_array[j + 1] * M_PI / 180.0;

				if ((angle_start <= angle) && (angle <= angle_end))
				{
					dist = 0.0;
					intensity = 0.0;
					break;
				}
			}
		}

		//距离是否在有效范围内 
		if (dist > cfg.range_max || dist < cfg.range_min)    
		{
			dist = 0.0;
			intensity = 0.0;
		}

		//角度是否在有效范围内 
		if ((angle >= outscan.config.min_angle) &&
			(angle <= outscan.config.max_angle))
		{
			NviLidarPoint point;
			point.angle = angle;
			point.range = dist;
			point.intensity = intensity;

			outscan.points.push_back(point);       
		}
	}

	//打印一下 
	//printf("out_count:%d,calc_count:%d,increse:%lf\n",outscan.points.size(),output_count,outscan.config.angle_increment);

	//如果固定角度分辨率 则resize 
	if(cfg.resolution_fixed)
	{
		outscan.points.resize(all_nodes_counts);
	}
}
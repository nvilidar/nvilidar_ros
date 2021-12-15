#pragma once

#include "nvilidar_config.h"
#include "nvilidar_def.h"
#include "nvilidar_protocol.h"
#include <cmath>
#include <vector>
#include "myconsole.h"
#include <iostream> 
#include <istream> 
#include <sstream>


//同步数据信息  相关信息 同步到雷达 
inline void LidarParaSync(Nvilidar_UserConfigTypeDef &cfg)
{
	cfg.storePara.samplingRate = (uint32_t)(cfg.sampling_rate *1000);		// * 1000
	cfg.storePara.angleOffset = (uint16_t)(cfg.angle_offset * 64 + 0.5);	//角度偏移 	实际与雷达的  64倍 U16 	
	cfg.storePara.isHasSensitive = cfg.sensitive;							//是否带有信号质量 
	cfg.storePara.aimSpeed = (uint16_t)(cfg.aim_speed *100 + 0.5);			//N Hz 实际与雷达的  100倍 U16 
	cfg.storePara.tailingLevel = cfg.tailing_level;							//拖尾等级 

	//ingnore array拆分 
	std::vector<float> elems;
	std::stringstream ss(cfg.ignore_array_string);
	std::string number;
	while (std::getline(ss, number, ',')) {
		elems.push_back(atof(number.c_str()));
	}
	cfg.ignore_array = elems;

	//看是否有需要过滤的数据 
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
	cfg.ip_addr = "192.168.1.200";	//192.168.1.200 为雷达默认IP 可更改 
	cfg.lidar_udp_port = 8100;				//8100为默认雷达传输用端口 不可更改 
	cfg.config_tcp_port = 8200;			//8200为默认雷达配置参数用端口 不可更改 
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
	cfg.tailing_level = 6;			//拖尾等级  
	cfg.angle_offset = 0.0;				//角度偏移 
	cfg.single_channel = false;			//单通道 
	cfg.ignore_array_string = "";				//过滤部分角度信息 

	LidarParaSync(cfg);
}

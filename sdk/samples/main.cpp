#define USE_TCP
//#define USE_SERIALPORT


#include <stdio.h>
#include <iostream>
#include "nvilidar_config.h"
#include "myconsole.h"
#include "mysignal.h"
#include "mytimer.h"

#ifdef USE_SERIALPORT
	#include "serial/nvilidar_serial.h"
	#include "nvilidar_driver_serialport.h"
#elif defined(USE_TCP)
	#include "socket/nvilidar_socket_udp_win.h"
	#include "nvilidar_driver_udp.h"
	#include "nvilidar_driver_net_config.h"
#endif

using namespace std;
using namespace nvilidar;

#if defined(_MSC_VER)
#pragma comment(lib, "nvilidar_driver.lib")
#endif


#ifdef USE_TCP
	int main()
	{
		printf(" _   ___      _______ _      _____ _____          _____ \n");
		printf("| \\ | \\ \\    / /_   _| |    |_   _|  __ \\   /\\   |  __ \\\n");
		printf("|  \\| |\\ \\  / /  | | | |      | | | |  | | /  \\  | |__) |\n");
		printf("| . ` | \\ \\/ /   | | | |      | | | |  | |/ /\\ \\ |  _  / \n");
		printf("| |\\  |  \\  /   _| |_| |____ _| |_| |__| / ____ \\| | \\ \\\n");
		printf("|_| \\_|   \\/   |_____|______|_____|_____/_/    \\_\\_|  \\ \\\n");
		printf("\n");
		fflush(stdout);

		//初始化信号 用于命令行退出  
		nvilidar::sigInit();

		
		//雷达接口  首先初始化 
		Nvilidar_UserConfigTypeDef  cfg;
		//获取默认参数  如需要修改 可以进行修改  
		LidarDefaultUserConfig(cfg);

		//=====================是否需要获取网络参数 （可修改及读取机器IP 不需要则注释掉）
		#if 0
			Nvilidar_NetConfigTypeDef net_cfg;
			nvilidar::LidarDriverNetConfig lidar_net_cfg(cfg);
			std::string ip,gate,mask;

			//建立连接  
			bool state = lidar_net_cfg.LidarNetConfigConnect();
			if(false == state)
			{
				nvilidar::console.warning("connect to config port error!");
				return 0;
			}

			//设置IP 
			ip = "192.168.1.201";
			gate = "192.168.1.1";
			mask = "255.255.255.0";
			state = lidar_net_cfg.LidarNetConfigWrite(ip,gate,mask);
			if(false == state)
			{
				nvilidar::console.warning("set ip error!");
				return 0;
			}
			delayMS(1000);
			//读取IP 
			state = lidar_net_cfg.LidarNetConfigRead(ip,gate,mask);
			if(false == state)
			{
				nvilidar::console.warning("read ip error!");
				return 0;
			}
			nvilidar::console.message("read net para:");
			nvilidar::console.message("ip:%s, gate:%s, mask:%s",ip.c_str(),gate.c_str(),mask.c_str());

			//断开连接  
			lidar_net_cfg.LidarNetConfigDisConnect();
		 #endif 

		//=====================正常雷达驱动  如不需要读写雷达参数  直接调用此接口即可 
		//初始化雷达 获取参数等等信息 
		nvilidar::LidarDriverUDP lidar(cfg);
		if (false == lidar.LidarInitialialize())		//初始化雷达  包括读参 配参 
		{
			return 0;
		}
		//启动雷达  
		bool ret = lidar.LidarTurnOn();


		//应答超时 
		uint32_t  no_response_times = 0;
		// 雷达点云图数据解析及处理
		while(ret && (nvilidar::isOK()))
		{
			LidarScan scan;

			if(lidar.LidarSamplingProcess(scan))
			{
				no_response_times = 0;

				for (size_t i = 0; i < scan.points.size(); i++)
				{
				  //float angle = scan.points.at(i).angle;
				  //float dis = scan.points.at(i).range;
				  //printf("a:%f,d:%f\n", angle, dis);
				}
			   nvilidar::console.message("Scan received[%llu]: %u ranges is [%f]Hz",
												 scan.stamp, (unsigned int)scan.points.size(),
												 1.0 / scan.config.scan_time);
			}
			else
			{
				no_response_times++;
				if(no_response_times >= 5)
				{
					no_response_times = 0;
					nvilidar::console.warning("Failed to get Lidar Data");

					break;
				}
			}

			delayMS(5);		//此处必须要加sleep 否则会占用超高cpu 
		}
		lidar.LidarTurnOff();      //停止扫描 
		nvilidar::console.message("lidar is stopping......");
		lidar.LidarCloseHandle();   //关闭连接 

		delayMS(100);

	   return 0;
	}

#endif 


#ifdef USE_SERIALPORT
	int main()
	{
		printf(" _   ___      _______ _      _____ _____          _____ \n");
		printf("| \\ | \\ \\    / /_   _| |    |_   _|  __ \\   /\\   |  __ \\\n");
		printf("|  \\| |\\ \\  / /  | | | |      | | | |  | | /  \\  | |__) |\n");
		printf("| . ` | \\ \\/ /   | | | |      | | | |  | |/ /\\ \\ |  _  / \n");
		printf("| |\\  |  \\  /   _| |_| |____ _| |_| |__| / ____ \\| | \\ \\\n");
		printf("|_| \\_|   \\/   |_____|______|_____|_____/_/    \\_\\_|  \\ \\\n");
		printf("\n");
		fflush(stdout);

		//初始化信号 用于命令行退出  
		nvilidar::sigInit();

		//应答超时 
		static uint32_t  no_response_times = 0;


		std::string port;       //选择的串口
		std::vector<NvilidarSerialPortInfo> ports = nvilidar::LidarDriverSerialport::getPortList();       //获取串口列表
		std::vector<NvilidarSerialPortInfo>::iterator it;

		//列表信息
		if (ports.empty())
		{
			nvilidar::console.show("Not Lidar was detected.");
			return 0;
		}
		else if (1 == ports.size())
		{
			it = ports.begin();
			port = (*it).portName;
		}
		else
		{
			int id = 0;
			for (it = ports.begin(); it != ports.end(); it++)
			{
				nvilidar::console.show("%d. %s  %s\n", id, it->portName.c_str(), it->description.c_str());
				id++;
			}
			while (nvilidar::isOK())
			{
				nvilidar::console.show("Please select the lidar port:");
				std::string number;
				std::cin >> number;

				//参数不合法 
				if ((size_t)atoi(number.c_str()) >= ports.size())
				{
					continue;
				}
				//参数配置 
				it = ports.begin();
				id = atoi(number.c_str());

				//查找  
				port = ports.at(id).portName;

				break;
			}
		}

		//雷达接口  首先初始化 
		Nvilidar_UserConfigTypeDef  cfg;

		//获取默认参数  如需要修改 可以进行修改  
		LidarDefaultUserConfig(cfg);
		//配置串口号 默认串口配置为空字符串 
		cfg.serialport_name = port;

		//初始化雷达 获取参数等等信息 
		nvilidar::LidarDriverSerialport lidar(cfg);
		if (false == lidar.LidarInitialialize())		//初始化雷达  包括读参 配参 
		{
			return 0;
		}
		//启动雷达  
		bool ret = lidar.LidarTurnOn();


		//    雷达点云图数据解析及处理
		while (ret && (nvilidar::isOK()))
		{
			LidarScan scan;

			if (lidar.LidarSamplingProcess(scan))
			{
				no_response_times = 0;

				for (size_t i = 0; i < scan.points.size(); i++)
				{
					//float angle = scan.points.at(i).angle;
					//float dis = scan.points.at(i).range;
					//printf("a:%f,d:%f\n", angle, dis);
				}
				nvilidar::console.message("Scan received[%llu]: %u ranges is [%f]Hz",
					scan.stamp, (unsigned int)scan.points.size(),
					1.0 / scan.config.scan_time);
			}
			else
			{
				no_response_times++;
				if (no_response_times >= 5)
				{
					no_response_times = 0;
					nvilidar::console.warning("Failed to get Lidar Data");

					break;
				}
			}

			delayMS(5);		//此处必须要加sleep 否则会占用超高cpu 
		}
		lidar.LidarTurnOff();      //停止扫描 
		nvilidar::console.message("lidar is stopping......");
		lidar.LidarCloseHandle();   //关闭连接 

		delayMS(100);

		return 0;
	}

#endif 

#include <stdio.h>
#include <iostream>
#include "nvilidar_driver_serialport.h"
#include "nvilidar_dataprocess.h"
#include "serial/nvilidar_serial.h"
#include "myconsole.h"
#include "mysignal.h"
#include "mytimer.h"

using namespace std;
using namespace nvilidar;

#if defined(_MSC_VER)
#pragma comment(lib, "nvilidar_driver.lib")
#endif

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

    std::string port;       //选择的串口
	std::vector<NvilidarSerialPortInfo> ports = nvilidar::LidarDriverSerialport::getPortList();       //获取串口列表
	std::vector<NvilidarSerialPortInfo>::iterator it;

    //列表信息
	if(ports.empty())
	{
		nvilidar::console.show("Not Lidar was detected.");
        return 0;
	}
    else if(1 == ports.size())
    {
		it = ports.begin();
		port = (*it).portName;
    }
    else
    {
        int id = 0;
        for (it = ports.begin(); it != ports.end(); it++)
        {
			nvilidar::console.show("%d. %s  %s\n", id, it->portName.c_str(),it->description.c_str());
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
	CircleDataInfoTypeDef node_circle;
	Nvilidar_DeviceInfo info;
	static uint32_t  no_response_times = 0;

	//获取默认参数  如需要修改 可以进行修改  
	LidarDefaultUserConfig(cfg);
	//配置串口号 默认串口配置为空字符串 
	cfg.serialport_name = port;

	//初始化雷达 获取参数等等信息 
	nvilidar::LidarDriverSerialport lidar(cfg);
	if (false == lidar.LidarInit())		//初始化雷达  包括读参 配参 
	{
		return 0;
	}
	//启动雷达  
	bool ret = lidar.StartScan();


//    雷达点云图数据解析及处理
    while(ret && (nvilidar::isOK()))
    {
        LidarScan scan;

        if(lidar.waitCircleResponse(node_circle))
        {
			no_response_times = 0;
			//点集格式转换 
			LidarSamplingData(cfg, node_circle, scan);

            for (size_t i = 0; i < scan.points.size(); i++)
            {
//              float angle = scan.points.at(i).angle;
//              float dis = scan.points.at(i).range;
//              printf("a:%f,d:%f\n", angle, dis);
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
    lidar.LidarStopScan();      //停止扫描 
    nvilidar::console.message("lidar is stopping......");
    lidar.LidarCloseHandle();   //关闭连接 

   return 0;
}

#include "CNviLidar.h"
#include "mysignal.h"
#include "console.h"

using namespace std;
using namespace nvilidar;

#if defined(_MSC_VER)
#pragma comment(lib, "nvilidar_driver.lib")
#endif

int main(int argc, char *argv[])
{
    CNviLidar lidar;


    printf(" _   ___      _______ _      _____ _____          _____ \n");
    printf("| \\ | \\ \\    / /_   _| |    |_   _|  __ \\   /\\   |  __ \\\n");
    printf("|  \\| |\\ \\  / /  | | | |      | | | |  | | /  \\  | |__) |\n");
    printf("| . ` | \\ \\/ /   | | | |      | | | |  | |/ /\\ \\ |  _  / \n");
    printf("| |\\  |  \\  /   _| |_| |____ _| |_| |__| / ____ \\| | \\ \\\n");
    printf("|_| \\_|   \\/   |_____|______|_____|_____/_/    \\_\\_|  \\ \\\n");
    printf("\n");
    fflush(stdout);

    std::string port;       //选择的串口

    //初始化信号
    nvilidar::SigInit();

    std::map<std::string, std::string> ports = CNvilidar::getLidarPortList();       //获取串口列表
    std::map<std::string, std::string>::iterator it;

    //列表信息
    if(1 == ports.size())
    {
        it = ports.begin();
        port = it->second;
    }
    else
    {
        int id = 0;
        for (it = ports.begin(); it != ports.end(); it++)
        {
            nvilidar::console.show("%d. %s\n", id, it->first.c_str());
            id++;
        }

        if (ports.empty())
        {
          nvilidar::console.show("Not Lidar was detected. Please enter the lidar serial port:");
          std::cin >> port;
        }
        else
        {
            while (nvilidar::isOK())
            {
                nvilidar::console.show("Please select the lidar port:");
                std::string number;
                std::cin >> number;

                if ((size_t)atoi(number.c_str()) >= ports.size()) {
                  continue;
                }

                it = ports.begin();
                id = atoi(number.c_str());

                while (id)
                {
                  id--;
                  it++;
                }

                port = it->second;
                break;
            }
        }
    }
    //是否OK
    if (!nvilidar::isOK())
    {
      return 0;
    }

    //先写初始值（全部可改  但是有些参数是连接后重新获取的）
    lidar.setSerialPort(port);
    lidar.setSerialBaudrate(921600);
    lidar.setAutoReconnect(true);//hot plug
    lidar.setMaxRange(64.0);
    lidar.setMinRange(0.1);
    lidar.setMaxAngle(180);
    lidar.setMinAngle(-180);
    lidar.setSampleRate(20);
    lidar.setScanFrequency(10.0);
    lidar.setReversion(true);
    lidar.setFixedResolution(false);

    //初始化  获取版本号及其它信息
    bool ret = lidar.Initialize();

    //开始启动雷达
    if(ret)
    {
        ret = lidar.LidarTurnOn();
    }

    //雷达点云图数据解析及处理
    while(ret && (nvilidar::isOK()))
    {
        LidarScan scan;

        if(lidar.LidarSamplingProcess(scan))
        {
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
            nvilidar::console.warning("Failed to get Lidar Data");
        }
    }

    lidar.LidarTurnOff();
    lidar.LidarCloseHandle();
    return 0;
}

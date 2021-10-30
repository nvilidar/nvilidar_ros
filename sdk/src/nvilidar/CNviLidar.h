#ifndef _CNVILIDAR_H_
#define _CNVILIDAR_H_

#include "lidar_serial_driver.h"
#include "lidar_scan.h"

//---定义库信息 VS系列的生成库文件  
#ifdef WIN32
	#ifdef nvilidar_IMPORTS
		#define NVILIDAR_API __declspec(dllimport)
	#else
		#ifdef nvilidarStatic_IMPORTS
			#define NVILIDAR_API
		#else
			#define NVILIDAR_API __declspec(dllexport)
		#endif // NVILIDAR_STATIC_EXPORTS
	#endif

#else
	#define NVILIDAR_API
#endif // ifdef WIN32


//---定义数据信息
#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The NVILIDAR SDK requires a C++ compiler to be built"
#endif
#endif
#define PropertyBuilderByName(type, name, access_permission)\
    access_permission:\
        type m_##name;\
    public:\
    inline void set##name(type v) {\
        m_##name = v;\
    }\
    inline type get##name() {\
        return m_##name;\
}\

class NVILIDAR_API CNviLidar
{
    //读信息
    PropertyBuilderByName(std::string, HardVer,
                          public) ///< 硬件版本号
    PropertyBuilderByName(std::string, SoftVer,
                          public) ///< 软件版本号
    PropertyBuilderByName(std::string, ProductName,
                          public) ///< 产品名称
    PropertyBuilderByName(std::string, SerialNum,
                          public) ///< 产品序列号

    //配置信息
    PropertyBuilderByName(float, MaxRange,
                          private) ///< 设置和获取激光最大测距范围
    PropertyBuilderByName(float, MinRange,
                          private) ///< 设置和获取激光最小测距范围
    PropertyBuilderByName(float, MaxAngle,
                          private) ///< 设置和获取激光最大角度, 最大值180度
    PropertyBuilderByName(float, MinAngle,
                          private) ///< 设置和获取激光最小角度, 最小值-180度
    PropertyBuilderByName(float, ScanFrequency,
                          private) ///< 设置和获取激光扫描频率(范围5HZ~12HZ)

    PropertyBuilderByName(bool, FixedResolution,
                          private) ///< 设置和获取激光是否是固定角度分辨率
    PropertyBuilderByName(bool, AutoReconnect,
                          private) ///< 设置异常是否自动重新连接

    PropertyBuilderByName(int, SerialBaudrate,
                          private) ///< 设置和获取激光通讯波特率
    PropertyBuilderByName(uint32_t, SampleRate,
                          private) ///< 设置和获取激光采样频率
    PropertyBuilderByName(int, AbnormalCheckCount,
                          private) ///< Maximum number of abnormal checks
    PropertyBuilderByName(bool, IsSingleConnect,
                          private) ///< 单通信
    PropertyBuilderByName(std::string, SerialPort,
                          private) ///< 设置和获取激光端口号
    PropertyBuilderByName(std::vector<float>, IgnoreArray,
                          private) ///< 设置和获取激光剔除点 2个为范围
    PropertyBuilderByName(bool, IsHasSensitive,
                          private) ///< 是否带信号质量
    PropertyBuilderByName(uint8_t, TrailingLevel,
                          private) ///< 拖尾等級
    PropertyBuilderByName(bool, Reversion, 
                          private); //倒置 反180度
    PropertyBuilderByName(bool, Inverted, 
                          private); //镜像 左右反相 
    PropertyBuilderByName(bool, SingleChannel, 
                          private); //单通道（此项未用）



public:
    CNviLidar();             //构造函数
    ~CNviLidar();            //析构函数
    bool LidarInitialize();          //初始化
    bool LidarCloseHandle();         //结束雷达 删除部分任务

    bool LidarTurnOn();         //打开雷达扫图功能
    bool LidarTurnOff();        //关闭雷达扫图功能

    bool LidarGetCfgInfo();             //获取雷达信息 包括并不限于 采样率  转速  带信号质量等
    bool LidarGetDevInfo();             //获取雷达设备信息

    bool LidarSamplingProcess(LidarScan &outscan);

    static std::map<std::string, std::string> getLidarPortList();    //获取雷达列表信息

private:
    nvilidar::LidarSerialDriver *driver;    //驱动信息
    bool isLidarScanning = false;           //是否正在扫描
    uint64_t last_node_time = 0;            //上一次时间戳
    int      lidar_model;                   //雷达型号
    uint32_t m_pointTime = 0;               //2个雷达点的时间间隔
    uint32_t m_packageTime = 0;             //零位包传送时
    uint32_t m_nodeCounts;                  //一圈的点数信息
    float    m_FrequencyOffset;             //偏移
    PackageConfigTypeDef driver_cfg;        //配置信息  写入配置类中

    bool LidarCheckAbnormal();          //检查雷达是否出现异常
    bool LidarStartConnect();           //连接并检测串口通信状态

    bool LidarSendStop();               //发送stop命令

protected:

};

#endif

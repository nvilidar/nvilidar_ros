#ifndef LIDAR_SERIAL_DIRVER_H
#define LIDAR_SERIAL_DRIVER_H

#include "serial/serial.h"
#include "lidar_def.h"
#include "lidar_protocol.h"
#include "timer.h"
#include "thread.h"
#include "locker.h"
#include <map>
#include <vector>

//数据模式
typedef void* (* pFUNC)(void *);

//包对齐
#if defined(_WIN32)
#pragma pack(1)
#endif

//雷达配置参数
struct  PackageConfigTypeDef
{
    uint8_t     IsSingleChannel;         //单通道通信
    uint8_t     IsHasSensitive;         //有信号质量信息
    uint16_t    aimSpeed;               //转速信息 x100
    uint32_t    samplingRate;           //采样率x1
    int16_t     angleOffset;            //角度偏移x64
    uint8_t     trailingLevel;          //拖尾等级
    device_info lidar_device_info;      //雷达设备信息
}__attribute__((packed));

//雷达信息
struct PackageStateTypeDef
{
    bool m_SerialOpen;              //串口开启标记
    bool m_Scanning;                //正在扫描出图
    uint8_t last_device_byte;       //上包接到的字节信息
}__attribute__((packed));

//共用体
union PackageBufTypeDef
{
    uint8_t buf[1200];
    node_package_qua        pack_qua;
    node_package_no_qua     pack_no_qua;
}__attribute__((packed));

//包信息
struct PackageInfoTypeDef
{
     uint16_t packageIndex;         //单包采样点索引位置信息
     PackageBufTypeDef  packageBuffer;    //包信息（实际内容）
     bool     packageErrFlag;       //包错误标记信息
     uint16_t packageCheckSumGet;   //校验值获取
     uint16_t packageCheckSumCalc;  //校验值计算
     uint8_t  packageFreq;          //雷达转速信息
     int16_t  packageTemp;          //雷达温度信息
     uint32_t packagePointTime;     //2点时间间隔
     uint16_t packageFirstAngle;    //起始采样角
     uint16_t packageLastAngle;     //结束采样角
     float    packageAngleDiffer;   //每2个点之间的角度差
     float    packageLastAngleDiffer; //最后一次算的2点角度的差值
     uint8_t  packagePointDistSize; //一个点对应的字节的大小信息
     bool     packageHas0CAngle;    //是否为0度角
     bool     packageHasTemp;       //是否当前位置为温度
     bool     packageHas0CFirst;    //第一个字节 判断是否是0度角
     bool     packageHasTempFirst;  //第一个字节 判断是否为温度信息
     uint16_t  package0CIndex;      //0度角索引（目前协议为非单独封包）
} __attribute__((packed));



#if defined(_WIN32)
#pragma pack()
#endif

namespace nvilidar
{
class LidarSerialDriver
{
public:
    LidarSerialDriver(PackageConfigTypeDef cfg);            //构造函数
    ~LidarSerialDriver();           //析构函数

    bool Connect(std::string portname,uint32_t baud = 921600);         //串口初始化
    bool DisConnect();      //关闭串口
    bool IsConnected();    //雷达是否连接


    //串口 版本号等信息
    virtual std::string getSDKVersion();                        //获取当前sdk版本号
    static  std::map<std::string, std::string> getPortList();        //获取机器串口列表
    virtual result_t StartScan(void);                           //启动扫图
    virtual result_t StopScan(void);                            //停止扫描
    virtual result_t LidarReset(void);                          //雷达是否复位
    virtual result_t LidarStopCmd(void);                        //停止命令

    virtual result_t SetIntensities(const uint8_t has_intensity, uint32_t timeout = DEFAULT_TIMEOUT);     //设置是否有信号质量
    virtual result_t GetDeviceInfo(device_info &info, uint32_t timeout = DEFAULT_TIMEOUT);
    //转速
    virtual result_t SetScanMotorSpeed(uint16_t frequency_add, uint16_t &ret_frequency,    //设置雷达目标转速
                                            uint32_t timeout = DEFAULT_TIMEOUT);
    virtual result_t SetSamplingRate(uint32_t rate_add, uint32_t &rate,
                                     uint32_t timeout = DEFAULT_TIMEOUT);   //雷达采样率增加
    virtual result_t SetTrailingLevel(uint8_t tral_set, uint8_t &tral,
                                           uint32_t  timeout = DEFAULT_TIMEOUT);
    //获取配置信息
    virtual result_t GetLidarCfg(lidar_get_info &info,
                                           uint32_t  timeout = DEFAULT_TIMEOUT);

    virtual result_t GetZeroOffsetAngle(int16_t &angle,       //设置0度偏移
                                        uint32_t timeout = DEFAULT_TIMEOUT);
    virtual result_t SetZeroOffsetAngle(int16_t angle_set, int16_t &angle,
                                        uint32_t timeout = DEFAULT_TIMEOUT); //读取0度偏移
    //保存参数
    virtual result_t SaveCfg(bool &flag,uint32_t timeout = DEFAULT_TIMEOUT);
    //抓包
    virtual result_t grabScanData(std::vector<node_info>&info,uint32_t timeout = DEFAULT_TIMEOUT);    //抓取数据
    virtual void disableDataGrabbing();             //禁止抓取数据
    //获取接当前扫描状态
    virtual uint32_t getScanState();                //获取扫描状态
    //获取接最后包的时间
    virtual uint32_t getPackageTime();              //获取当前整包传送的时间
    //获取0度已传输时间
    virtual uint32_t getZeroIndex();                //获取0度索引 
private:
    result_t SendSerial(const uint8_t *data, size_t size);     //发送串口接口  私有类
    result_t RecvSerial(const uint8_t *data, size_t size);     //接收串口数据  私有类
    result_t RecvLenSerial(size_t *len);                       //获取可用字节长度
    virtual result_t SendCommand(uint8_t cmd, const void *payload = NULL,size_t payloadsize = 0);
    virtual result_t WaitResponseHeader(lidar_ans_header *header,uint32_t timeout = DEFAULT_TIMEOUT);       //包头数据接收
    virtual result_t WaitResponseData(uint8_t *data, size_t length, uint32_t timeout = DEFAULT_TIMEOUT);    //任意数据接收  外加超时
    virtual result_t WaitPointSinglePackData(std::vector<node_info> &info,uint32_t timeout = DEFAULT_TIMEOUT);     //获取点云数据(单包接口)
    virtual result_t WaitPointCircleData(void);                 //获取一圈点
    int ScanThread();                        //雷达扫描

    virtual result_t createThread();                            //创建线程 出图时用


    serial::Serial      *serialport;        //串口子类
    Thread              _thread;			//线程id
    Event               _dataEvent;			//数据同步事件
    Locker              _lock;              //线程锁

    //-----------------------变量----------------------------
    PackageInfoTypeDef  packageInfo;        //包信息

    PackageStateTypeDef   lidar_state;        //雷达状态
    PackageConfigTypeDef  lidar_cfg;     //雷达型号
    lidar_get_info        lidar_info;    //雷达信息

    std::vector<node_info>  circle_node_points;   //一圈点云图数据

    uint32_t    m_packageTime = 0;          //一包数据的传输时间
    uint32_t    m_pointTime = 0;            //2个雷达点的时间间隔
    uint32_t    m_0cIndex = 0;              //0度所用的index
    int32_t    m_last0cIndex = 0;              //0度所用的index
    uint32_t    m_differ0cIndex = 0;              //0度所用的index
    bool        m_first_circle_finish = false;  //first circle finish,case calc fault
};
}

#endif

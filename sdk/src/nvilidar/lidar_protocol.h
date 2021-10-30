#ifndef _LIDAR_PROTOCOL_H_
#define _LIDAR_PROTOCOL_H_

#include <stdint.h>
#include <vector>

//其它
#define LIDAR_CMD_STOP                      0x65        //停止雷达
#define LIDAR_CMD_SCAN                      0x60        //开始扫描雷达
#define LIDAR_CMD_RESET                     0x40        //复位雷达
#define LIDAR_CMD_GET_DEVICE_INFO           0xB2        //获取设备信息 包括序列号 版本号等信息

#define LIDAR_CMD_GET_LIDAR_CFG             0xDA        //获取雷达部分配置信息

#define LIDAR_CMD_SAVE_LIDAR_PARA           0xD6        //保存雷达参数信息

#define LIDAR_CMD_GET_AIMSPEED              0xDA        //读目标转速
#define LIDAR_CMD_SET_AIMSPEED              0x27        //写目标转速

#define LIDAR_CMD_GET_SAMPLING_RATE         0xDA        //读采样率
#define LIDAR_CMD_SET_SAMPLING_RATE         0xCA        //写采样率

#define LIDAR_CMD_GET_TAILING_LEVEL         0xDA        //读角度偏移
#define LIDAR_CMD_SET_TAILING_LEVEL         0xCB        //写角度偏移

#define LIDAR_CMD_GET_ANGLE_OFFSET          0xC5        //读角度偏移
#define LIDAR_CMD_SET_ANGLE_OFFSET          0xC4        //写角度偏移

#define LIDAR_CMD_SET_HAVE_INTENSITIES      0x50        //有信号质量
#define LIDAR_CMD_SET_NO_INTENSITIES        0x51        //设置没有信号质量

//长短命令头字节 尾字节
#define LIDAR_START_BYTE_LONG_CMD           0x40        //长命令  头字节
#define LIDAR_START_BYTE_SHORT_CMD          0xFE        //短命令  头字节
#define LIDAR_END_CMD                       0xFF        //包尾字节
//一包最多的点数信息
#define LIDAR_PACK_MAX_POINTS               256


#define LIDAR_PACKAGE_HEAD_SIZE             12          //包头（扫图时）

#define LIDAR_POINT_HEADER                0x55AA        //包头信息


#define LIDAR_RESP_MEASUREMENT_CHECKBIT    (0x1)     //角度标记位是否有效
#define LIDAR_ANGULDAR_RESOLUTION          64           //角分辨率 即多少数表示为1度角

//idx代表不同数据
enum
{
    PACK_INDEX_NONE = 0,                //本包3 4字节 无其它意义
    PACK_INDEX_TEMP = 1,
};


//包对齐
#if defined(_WIN32)
#pragma pack(1)
#define __attribute__(...)
#endif

//单点结构体信息
struct node_info {
    uint8_t    lidar_angle_zero_flag;     //检测到0位角标记
    uint16_t   lidar_quality;             //信号质量
    float      lidar_angle;               //测距点角度
    uint16_t   lidar_distance;            //当前测距点距离
//    uint64_t   lidar_stamp;               //时间戳
    float      lidar_speed;               //扫描频率
    float      lidar_temper;              //雷达 温度值
    uint32_t   lidar_point_time;          //2点时间间隔
    uint8_t    debugInfo;
    uint8_t    index;
    uint8_t    error_package;
} __attribute__((packed)) ;

//包信息(带信号质量)
struct PackageNode_Quality
{
    uint16_t PakageSampleQuality;
    uint16_t PakageSampleDistance;
} __attribute__((packed));

//包信息（不带信号质量）
struct PackageNode_NoQualiry
{
    uint16_t PakageSampleDistance;
}__attribute__((packed));


//包数目 加信号质量
struct node_package_qua {
    uint16_t  package_Head;
    uint8_t   package_CT;
    uint8_t   nowPackageNum;
    uint16_t  packageFirstSampleAngle;
    uint16_t  packageLastSampleAngle;
    uint16_t  checkSum;
    PackageNode_Quality  packageSample[LIDAR_PACK_MAX_POINTS];
} __attribute__((packed)) ;

//包信息 不加信号质量
struct node_package_no_qua {
    uint16_t  package_Head;
    uint8_t   package_CT;
    uint8_t   nowPackageNum;
    uint16_t  packageFirstSampleAngle;
    uint16_t  packageLastSampleAngle;
    uint16_t  checkSum;
    PackageNode_NoQualiry  packageSample[LIDAR_PACK_MAX_POINTS];
} __attribute__((packed)) ;

//设备信息
struct device_info {
    uint8_t SW_V[2];            //软件版本号
    uint8_t HW_V[2];            //硬件版本号
    uint8_t MODEL_NUM[5];       //雷达型号
    uint8_t serialnum[16];      //序列号
} __attribute__((packed)) ;

//应答时的数据结构模式
struct lidar_ans_header {
  uint8_t  startByte;       //起始命令 1byte
  uint8_t  cmd;             //命令字   1Byte
  uint16_t length;          //长度    2byte
} __attribute__((packed));

//发送时的数据结构模式
struct lidar_send_header {
  uint8_t  startByte;       //起始命令 1byte
  uint8_t  cmd;             //命令字   1Byte
  uint16_t length;          //长度    2byte
} __attribute__((packed));

//读雷达信息模式
struct lidar_get_info {
  uint16_t  apdValue;        //Apd值
  uint32_t samplingRate;    //采样率
  uint16_t aimSpeed;        //目标转速
  uint8_t  trailLevel;      //拖尾等级
  uint8_t  hasSensitive;    //是否有信号质量
}__attribute__((packed));

//应答时的包尾数据模式
struct lidar_ans_tail {
  uint8_t  crc;             //校验值
  uint8_t  endByte;         //最后一字节 0xFF
} __attribute__((packed));

#if defined(_WIN32)
#pragma pack()
#endif


#endif

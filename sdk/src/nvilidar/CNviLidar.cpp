#include "CNviLidar.h"
#include "console.h"
#include "mystring.h"
#include "angles.h"

//构造函数
CNviLidar::CNviLidar()
{
    //基本信息
    m_SoftVer           = "0.0";
    m_HardVer           = "0.0";
    m_ProductName       = "Nvilidar";
    m_SerialNum         = "0000000000000000";

    //配置信息
    m_SerialPort        = "";
    m_SerialBaudrate    = 921600;   //波特率
    m_FixedResolution   = false;    //是否固定角分辨率
    m_AutoReconnect     = true;
    m_MaxAngle          = 180.f;
    m_MinAngle          = -180.f;
    m_MaxRange          = 64.0;
    m_MinRange          = 0.00;
    m_SampleRate        = 20000;
    m_ScanFrequency     = 10.0;     //hz  电机转速
    isLidarScanning     = false;    //雷达是否正在扫描 置false
    m_nodeCounts        = (uint32_t)(m_SampleRate/m_ScanFrequency);
    m_FrequencyOffset   = 0.0;      //频率偏移
    m_AbnormalCheckCount = 4;
    m_IgnoreArray.clear();
    m_pointTime         = 1e9 / m_SampleRate;
    m_packageTime       = 0;        //零位包传送时间
    m_IsSingleConnect   = false;    //单通信功能使
    m_IsHasSensitive    = false;    //带有信号质量
    m_TrailingLevel     = 6;        //默认拖尾过滤等级 1~20  1表示最强
    m_Inverted          = false;    //镜像 置为false
    m_Reversion         = false;    //180度
    m_SingleChannel     = false;    //单通道  目前未用 

    //driver cfg
    driver_cfg.aimSpeed = m_ScanFrequency*100;     //x100
    driver_cfg.angleOffset = m_FrequencyOffset*64;     //角度偏移
    driver_cfg.samplingRate = m_SampleRate;
    driver_cfg.trailingLevel = m_TrailingLevel;
    driver_cfg.IsHasSensitive = m_IsHasSensitive;   //信号质量
    driver_cfg.IsSingleChannel = m_IsSingleConnect; //单通道 目前暂时未用 后续会用


    last_node_time      = getTime();
    lidar_model = NVILIDAR_VP300;   //默认vp300型号
}

//析构函数
CNviLidar::~CNviLidar()
{
    LidarCloseHandle();
}

//获取雷达串口列表信息
std::map<std::string, std::string> CNviLidar::getLidarPortList()
{
    return  nvilidar::LidarSerialDriver::getPortList();
}

//雷达接口
bool CNviLidar::LidarTurnOn()
{
    result_t ans;
    //如果雷达正在运行  则返回状态
    if((driver->getScanState())&&isLidarScanning)
    {
        return true;
    }

    //启动雷达抛图模式
    ans = driver->StartScan();
    if(!IS_OK(ans))
    {
        driver->StopScan();
        nvilidar::console.error("[CNviLidar] Failed to start scan mode: %x", ans);
        isLidarScanning = false;

        return false;
    }

    //检测超时等信息
    if (LidarCheckAbnormal())
    {
        driver->StopScan();
        nvilidar::console.error("[CNviLidar] Failed to turn on the Lidar");
        isLidarScanning = false;

        return false;
    }

    isLidarScanning = true;
    //启动成功
    nvilidar::console.message("[NVILIDAR INFO] Now NVILIDAR is scanning ......");
    fflush(stdout);

    return true;
}

//关闭雷达扫图功能
bool CNviLidar::LidarTurnOff()
{
    if (isLidarScanning)
    {
        driver->StopScan();        //停止扫描
        nvilidar::console.message("[NVILIDAR INFO] Now NVILIDAR Scanning has stopped ......");
    }

    isLidarScanning = false;
    return true;
}

//检查雷达出图是否出现异常
bool CNviLidar::LidarCheckAbnormal()
{
    return false;
}

//雷达建立连接 并看是否存在问题
bool CNviLidar::LidarStartConnect()
{
    driver = new nvilidar::LidarSerialDriver(driver_cfg);

    //雷达连接是否成功
    if(driver->IsConnected())
    {
        return true;
    }

    //建立连接
    bool state = driver->Connect(m_SerialPort.c_str(), m_SerialBaudrate);  //启动串口接口

    

    //检查建立连接是否成功
    if (!state)
    {
        nvilidar::console.error("[NvidLidar] Error, cannot bind to the specified serial port %s",
                             m_SerialPort.c_str());
        return false;
    }

    return true;
}

//雷达发停止命令
bool CNviLidar::LidarSendStop()
{
    return driver->LidarStopCmd();     //发停止命令
}

//获取雷达设备信息
bool CNviLidar::LidarGetDevInfo()
{
    result_t ans;
    device_info     dev_info;       //设备型号等信息

    //雷达连接是否成功
    if(! driver->IsConnected())
    {
        return false;
    }

    //1-获取型号等信息
    ans = driver->GetDeviceInfo(dev_info);
    if(!IS_OK(ans))
    {
        return false;
    }

    //生成字符信息
    m_SoftVer = formatString("V%d.%d",dev_info.SW_V[0],dev_info.SW_V[1]);
    m_HardVer = formatString("V%d.%d",dev_info.HW_V[0],dev_info.HW_V[1]);
    m_ProductName = formatString("%s",dev_info.MODEL_NUM);
    m_SerialNum = formatString("%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d%01d",
                               dev_info.serialnum[0],dev_info.serialnum[1],dev_info.serialnum[2],dev_info.serialnum[3],
                               dev_info.serialnum[4],dev_info.serialnum[5],dev_info.serialnum[6],dev_info.serialnum[7],
                               dev_info.serialnum[8],dev_info.serialnum[9],dev_info.serialnum[10],dev_info.serialnum[11],
                               dev_info.serialnum[12],dev_info.serialnum[13],dev_info.serialnum[14],dev_info.serialnum[15]);

    return true;
}

//检测连接等等信息
bool CNviLidar::LidarGetCfgInfo()
{
    result_t ans;
    lidar_get_info  lidar_info;     //雷达信息 从协议获取来
    device_info     dev_info;       //设备型号等信息

    //雷达连接是否成功
    if(! driver->IsConnected())
    {
        return false;
    }


    //1-获取配置信息
    ans = driver->GetLidarCfg(lidar_info);
    if(!IS_OK(ans))
    {
        return false;
    }

    //配置信息 get
    m_ScanFrequency = (float)(lidar_info.aimSpeed)/100.0;      //目标转速
    m_TrailingLevel = lidar_info.trailLevel;                //拖尾过滤等级
    m_IsHasSensitive = lidar_info.hasSensitive;             //是否有信号质量
    m_SampleRate =  lidar_info.samplingRate;                //采样率
    m_pointTime = 1e9 / m_SampleRate;                       //求得2点之间的时间间隔
    m_nodeCounts   = (uint32_t)(m_SampleRate/m_ScanFrequency);  //更新采样点数

    return true;
}

//采样信息
bool CNviLidar::LidarSamplingProcess(LidarScan &outscan)
{
    std::vector<node_info>PointLidar;
    size_t all_nodes_counts = m_nodeCounts;     //当前点数信息

    uint64_t tim_scan_start = getTime();        //获取起始时间
    uint64_t startTs = tim_scan_start;          //获取起始时间
    result_t op_result =  driver->grabScanData(PointLidar);
    uint64_t tim_scan_end = getTime();

    //清空输出缓存
    outscan.points.clear();

    //如果超时  则进行解包等等操作
    if(IS_OK(op_result))
    {
        uint32_t count = PointLidar.size();

        //超出了 不计算  有误
//        if((count >(m_nodeCounts + m_nodeCounts /10) ) || (count < (m_nodeCounts - m_nodeCounts / 10)))
//        {
//            return false;
//        }

        //是否固定的角分辨率
        if (m_FixedResolution)
        {
            all_nodes_counts = m_nodeCounts;

        }
        else
        {
            all_nodes_counts = count;
        }
        //最大角度和最小角度
        if (m_MaxAngle < m_MinAngle)
        {
            float temp = m_MinAngle;
            m_MinAngle = m_MaxAngle;
            m_MaxAngle = temp;
        }

        //初步计算时间
        uint64_t scan_time = m_pointTime * (count - 1);     //浏览一圈所需要的时间

        tim_scan_end -= driver->getPackageTime();           //减去传输一包所用时间
        tim_scan_end -= driver->getZeroTransTime();         //减于0位部分时间

        tim_scan_start = tim_scan_end -  scan_time ;        //推算的起始时间  减去一圈点的时间

        //判断起始时间 小于计时时间
        if (tim_scan_start < startTs)
        {
            tim_scan_start = startTs;
            tim_scan_end = tim_scan_start + scan_time;
        }


        int counts = all_nodes_counts * ((m_MaxAngle - m_MinAngle) / 360.0f);   //计算点数信息

        outscan.stamp = tim_scan_start;                 //起始时间 起始时间戳
        outscan.config.max_angle = angles::from_degrees(m_MaxAngle);        //最大角度 转弧度
        outscan.config.min_angle = angles::from_degrees(m_MinAngle);        //最小角度 转弧度
        outscan.config.angle_increment = (outscan.config.max_angle -        //算出点的角度差值
                                          outscan.config.min_angle) /
                                         (double)(counts - 1);
        outscan.config.scan_time = static_cast<float>(1.0 * scan_time / 1e9);   //一圈点时间（转成s）
        outscan.config.time_increment = outscan.config.scan_time / (double)(counts - 1); //一个点时间（转成s）
        outscan.config.min_range = m_MinRange;          //距离过滤最小值
        outscan.config.max_range = m_MaxRange;          //距离过滤最大值

        float dist = 0.0;
        float angle = 0.0;
        float intensity = 0.0;
        unsigned int i = 0;

        for (; i < count; i++)
        {
            //变换单位 求得真实值
            dist = static_cast<float>(PointLidar.at(i).lidar_distance / 1000.f);
            intensity = static_cast<float>(PointLidar.at(i).lidar_quality);
            angle = static_cast<float>(PointLidar.at(i).lidar_angle);
            angle = angles::from_degrees(angle);
            angle = 2 * M_PI - angle;

            //Rotate 180 degrees or not
            if (m_Reversion) 
            {
                angle = angle + M_PI;
            }

            //Is it counter clockwise
            if (m_Inverted) 
            {
                angle = 2 * M_PI - angle;
            }

            //求取角度 转成-pi ~ pi
            angle = angles::normalize_angle(angle); //转成-pi ~ pi

            //忽略点（事先配置好哪个角度的范围）
            if (m_IgnoreArray.size() != 0)
            {
                for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2)
                {
                    if ((angles::from_degrees(m_IgnoreArray[j]) <= angle) &&
                      (angle <= angles::from_degrees(m_IgnoreArray[j + 1])))
                    {
                        dist = 0.0;
                        intensity = 0.0;
                        break;
                    }
                }
            }

            //距离是否在有效范围内
            if (dist > m_MaxRange || dist < m_MinRange)     //超出限制部分  清0
            {
                dist = 0.0;
                intensity = 0.0;
            }

            //角度在区间范围内
            if ((angle >= outscan.config.min_angle) &&
                    (angle <= outscan.config.max_angle))
            {
                LidarPoint point;
                point.angle = angle;
                point.range = dist;
                point.intensity = intensity;

                outscan.points.push_back(point);        //追加进去
            }
        }

        if (m_FixedResolution)
        {
            outscan.points.resize(all_nodes_counts);        //点数信息
        }
        return true;
    }
    else
    {
        if (IS_FAIL(op_result))
        {
          // Error? Retry connection
        }
    }


    return false;
}

//系统初始化
bool CNviLidar::LidarInitialize()
{
  if (!LidarStartConnect())
  {
    nvilidar::console.error("Error initializing NVILIDAR scanner.");
    return false;
  }

  //发停止命令
  LidarSendStop();

  delay(300);

  //获取雷达信息
  if(!LidarGetDevInfo())
  {
      nvilidar::console.warning("Error initializing NVILIDAR scanner.Failed to get Lidar Device Info.");
      return false;
  }
  //打印设备信息
  nvilidar::console.show("\nlidar device info:");
  nvilidar::console.show("lidar name:%s",m_ProductName.c_str());
  nvilidar::console.show("lidar soft version:%s",m_SoftVer.c_str());
  nvilidar::console.show("lidar hard version:%s",m_HardVer.c_str());
  nvilidar::console.show("lidar serialnumber:%s",m_SerialNum.c_str());

  //获取雷达配置参数
  if (!LidarGetCfgInfo())
  {
    nvilidar::console.warning("Error initializing NVILIDAR scanner.Failed to get Lidar Config Info.");
    return false;
  }
  //打印配置信息
  nvilidar::console.show("\nlidar config info:");
  nvilidar::console.show("lidar samplerate :%d",m_SampleRate);
  nvilidar::console.show("lidar frequency :%.1f",m_ScanFrequency);
  nvilidar::console.show("lidar sesitive :%s",m_IsHasSensitive?"yes":"no");
  nvilidar::console.show("lidar trail filter level :%d",m_TrailingLevel);

  delay(50);

  return true;
}

//删除对应任务
bool CNviLidar::LidarCloseHandle()
{
    if(driver)
    {
        driver->DisConnect();
        delete  driver;
        driver = nullptr;
    }

    isLidarScanning = false;

    return true;
}

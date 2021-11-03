#include "lidar_serial_driver.h"
#include "serial/serial.h"


namespace nvilidar
{
//构造函数
LidarSerialDriver::LidarSerialDriver(PackageConfigTypeDef cfg)
{
    lidar_state.m_SerialOpen = false;       //默认串口关闭
    lidar_state.m_Scanning = false;         //默认扫描接口关闭

    m_packageTime  = 0;                     //2包数据传输时间间隔清0

    if(cfg.IsHasSensitive)
    {
        packageInfo.packagePointDistSize = 4;   //根据字节大小判断
    }
    else
    {
        packageInfo.packagePointDistSize = 2;   //根据字节大小判断
    }

    m_pointTime = 1e9/cfg.samplingRate;


    lidar_cfg = cfg;                   //配置参数生效
}

//析构函数
LidarSerialDriver::~LidarSerialDriver()
{
    DisConnect();
}

//启动雷达串口
bool LidarSerialDriver::Connect(std::string portname,uint32_t baud)
{
    serialport = new serial::Serial(portname,baud,serial::Timeout::simpleTimeout(1000));

    if(serialport->isOpen())
    {
        lidar_state.m_SerialOpen = true;
        //获取字节传输时间
        // m_byte_trans_delay = serialport->getByteTime();     //获取2字节传输时间

        return true;
    }
    else
    {
        lidar_state.m_SerialOpen = false;

        return false;
    }
}

//关闭串口
bool LidarSerialDriver::DisConnect()
{
    lidar_state.m_Scanning = false;
    //printf("disconnect... scanid = %d",thread_scan_id);
    //删除线程
    _thread.join();

    if(serialport)
    {
        //printf("delete serial \r\n");
        serialport->close();        //关闭串口
        delete serialport;          //删除子类
        serialport = nullptr;
    }
    return true;
}

//雷达是否连接
bool LidarSerialDriver::IsConnected()
{
    if((lidar_state.m_SerialOpen) && (serialport != NULL))
    {
        return true;
    }
    return false;
}

//发送串口
result_t LidarSerialDriver::SendSerial(const uint8_t *data, size_t size)
{
  if (!lidar_state.m_SerialOpen)
  {
    return RESULT_FAIL;
  }

  if (data == NULL || size == 0) {
    return RESULT_FAIL;
  }

  size_t r;

  while (size) {
    r = serialport->write(data, size);

    if (r < 1) {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;
  }

  return RESULT_OK;
}

//接收串口 有效数据 长度
result_t LidarSerialDriver::RecvLenSerial(size_t *len)
{
    if (!lidar_state.m_SerialOpen)
    {
      return RESULT_FAIL;
    }

    *len = serialport->available();     //读缓存内还有多少个字节

    return RESULT_OK;
}

//接收串口 有效数据 数据包
result_t LidarSerialDriver::RecvSerial(const uint8_t *data, size_t size)
{
    if (!lidar_state.m_SerialOpen)
    {
      return RESULT_FAIL;
    }

    size_t r;

    while (size) {
      r = serialport->read((uint8_t *)data, size);

      if (r < 1) {
        return RESULT_FAIL;
      }

      size -= r;
      data += r;
    }

    return RESULT_OK;
}

result_t LidarSerialDriver::createThread()
{
    _thread = CLASS_THREAD(LidarSerialDriver,ScanThread);

    if (_thread.getHandle() == 0)
    {
        lidar_state.m_Scanning = false;
        return RESULT_FAIL;
    }
    lidar_state.m_Scanning = true;
    return RESULT_OK;
}

//雷达发送数据
result_t LidarSerialDriver::SendCommand(uint8_t cmd, const void *payload,size_t payloadsize)
{
    uint8_t pkt_header[sizeof(lidar_send_header)];
    lidar_send_header *header = reinterpret_cast<lidar_send_header * >(pkt_header);
    uint8_t checksum = 0;
    uint8_t pkt_tail = LIDAR_END_CMD;     //包尾

    //串口未打开 返回失败
    if (!lidar_state.m_SerialOpen)
    {
        return RESULT_FAIL;
    }

    if (payloadsize && payload)   //起始字节  根据有没有内容  来看是长命令字还是短命令字
    {
        //长命令
        header->startByte = LIDAR_START_BYTE_LONG_CMD;
        header->cmd = cmd;
        header->length = payloadsize;

        //计算校验值
        for (size_t pos = 0; pos < payloadsize; ++pos)
        {
         checksum ^= ((uint8_t *)payload)[pos];
        }

        uint16_t sizebyte = (uint8_t)(payloadsize);

        //开始进行发送
        SendSerial(pkt_header, 4) ;       //包头 长度信息
        SendSerial((const uint8_t *)payload, sizebyte);   //发送数据信息
        SendSerial(&checksum, 1);         //校验值
        SendSerial(&pkt_tail, 1);         //校验值
    }
    else
    {
        //短命令
        header->startByte = LIDAR_START_BYTE_SHORT_CMD;
        header->cmd = cmd;
        SendSerial(pkt_header, 2) ;
    }

    return RESULT_OK;
}

//接包头数据 (包头信息  超时时间 )
result_t LidarSerialDriver::WaitResponseHeader(lidar_ans_header *header,uint32_t timeout)
{
    int  recvPos     = 0;
    uint32_t startTs = getms();        //获取当前ms数
    uint8_t  recvBuffer[sizeof(lidar_ans_header)];
    uint8_t  *headerBuffer = reinterpret_cast<uint8_t *>(header);
    uint32_t waitTime = 0;
    lidar_state.last_device_byte = 0x00;

    while ((waitTime = getms() - startTs) <= timeout)
    {
        size_t recvSize = 0;
        //读字节剩余长度信息
        result_t ans = RecvLenSerial(&recvSize);  //读缓存内已有字节长度信息
        //超时未接全包头 则认为有问题
        if (!IS_OK(ans))
        {
          return ans;
        }
        //接到了多少个字节（超过包头字长 则只保留包头字长）
        if (recvSize > sizeof(lidar_ans_header))
        {
          recvSize = sizeof(lidar_ans_header);
        }
        //接收数据大于0  才开始读数据处理
        if(recvSize > 0)
        {
            //获取字节包数据
            ans = RecvSerial(recvBuffer, recvSize);

            //未获取到正常数据  则认为失败
            if (IS_FAIL(ans))
            {
                return RESULT_FAIL;
            }
            //获取信息
            for (size_t pos = 0; pos < recvSize; ++pos)
            {
                uint8_t currentByte = recvBuffer[pos];

                //position = 0  接到第一字节为止 才停
                if(0 == recvPos)
                {
                    if (currentByte != LIDAR_START_BYTE_LONG_CMD)      //根据命令字来区分 收到的是打包数据 还是其它
                    {
                        recvPos = 0;
                        lidar_state.last_device_byte = currentByte;
                        continue;
                    }
                }

                headerBuffer[recvPos++] = currentByte;
                lidar_state.last_device_byte = currentByte;

                if (recvPos == sizeof(lidar_ans_header))
                {
                    return RESULT_OK;
                }
            }
        }
    }

    return RESULT_FAIL;
}

//获取普通数据应答(外加超时信息)
result_t LidarSerialDriver::WaitResponseData(uint8_t *data,size_t length,uint32_t timeout)
{
    uint32_t startTs = getms();        //获取当前ms数
    uint32_t waitTime;
    size_t recvSize = 0;
    lidar_ans_tail lidar_tail;
    result_t ans = RESULT_FAIL;
    uint8_t  crc = 0;                   //校验值

    //查看超时情况
    while ((waitTime = getms() - startTs) <= timeout)
    {
        //读字节剩余长度信息
        result_t ans = RecvLenSerial(&recvSize);  //读缓存内已有字节长度信息
        //超时未接全包头 则认为有问题
        if (!IS_OK(ans))
        {
            return ans;
        }

        //超过部分来读(要把包尾数据一起读出来 )
        if(recvSize >= length + sizeof(lidar_ans_tail))
        {
            recvSize = length + sizeof(lidar_ans_tail);
            break;
        }
    }
    //超时
    if(recvSize < length + sizeof(lidar_ans_tail))       //超时了 字节还没接够  则返回超时状态信息
    {
        return RESULT_TIMEOUT;
    }
    //读数据
    ans = RecvSerial(data,length);
    //校验计算比对
    ans = RecvSerial((uint8_t *)&lidar_tail,sizeof(lidar_ans_tail));

    //校验计算
    crc = data[0];  //取到第一个字节
    for(size_t i = 1; i<length; i++)
    {
        crc ^= data[i];
    }

    if(crc != lidar_tail.crc)
    {
        ans = RESULT_FAIL;
    }

    return ans;
}

//获取SDK版本号
std::string LidarSerialDriver::getSDKVersion()
{
    return SDKVerision;
}

//获取串口列表信息
std::map<std::string, std::string> LidarSerialDriver::getPortList()
{
    std::vector<serial::PortInfo> lst = serial::list_ports();
    std::map<std::string, std::string> ports;

#if defined (_WIN32)
    for (std::vector<serial::PortInfo>::iterator it = lst.begin(); it != lst.end(); it++) {
      std::string port = (*it).description;
      ports[port] = (*it).port;
    }
#else
    for (std::vector<serial::PortInfo>::iterator it = lst.begin(); it != lst.end(); it++)
    {
        //printf("port:%s,des:%s\n",(*it).port.c_str(),(*it).description.c_str());
        //usb check
        if((*it).port.find("ttyACM") != std::string::npos)      //linux ttyacm
        {
            std::string port = (*it).description;
            ports[port] = (*it).port;
        }
    }
#endif

    return ports;
}

//设置雷达是否带信号质量信息
result_t LidarSerialDriver::SetIntensities(const uint8_t has_intensity, uint32_t timeout)
{
    result_t  ans;
    uint8_t   cmd;

    //串口有没有开
    if (!lidar_state.m_SerialOpen)
    {
        return RESULT_FAIL;
    }

    //停止点云图数据抓取
    disableDataGrabbing();
    //刷新串口 并做接收处理
    serialport->flush();
    {
        ScopedLocker l(_lock);      //关闭锁
        cmd = (has_intensity ? LIDAR_CMD_SET_HAVE_INTENSITIES : LIDAR_CMD_SET_NO_INTENSITIES);
        //发送命令
        if ((ans = SendCommand(cmd)) != RESULT_OK)
        {
            return ans;
        }

        lidar_ans_header response_header;

        //接收包头
        if ((ans = WaitResponseHeader(&response_header, timeout)) != RESULT_OK)
        {
            return ans;
        }

        //长度比包头还小 认定为不合理
        if (response_header.length < sizeof(lidar_ans_header))
        {
            return RESULT_FAIL;
        }
        //命令字对不上  认为无效
        if(response_header.cmd != cmd)
        {
            return RESULT_FAIL;
        }
        //接剩下来的字节
        result_t ans = WaitResponseData(reinterpret_cast<uint8_t *>(&lidar_cfg.IsHasSensitive),sizeof(lidar_cfg.IsHasSensitive),timeout);

        if(IS_OK(ans))
        {
            return RESULT_OK;
        }
    }

    return RESULT_FAIL;
}

//获取设备类型信息
result_t LidarSerialDriver::GetDeviceInfo(device_info &info, uint32_t timeout)
{
    result_t  ans;

    //串口有没有开
    if (!lidar_state.m_SerialOpen)
    {
        return RESULT_FAIL;
    }

    //停止点云图数据抓取
    disableDataGrabbing();
    //刷新串口 并做接收处理
    serialport->flush();
    {
        ScopedLocker l(_lock);      //关闭锁
        //发送命令
        if ((ans = SendCommand(LIDAR_CMD_GET_DEVICE_INFO)) != RESULT_OK)
        {
            return ans;
        }

        lidar_ans_header response_header;

        //接收包头
        if ((ans = WaitResponseHeader(&response_header, timeout)) != RESULT_OK)
        {
            return ans;
        }

        //长度比包头还小 认定为不合理
        if (response_header.length < sizeof(lidar_ans_header))
        {
            return RESULT_FAIL;
        }
        //命令字对不上  认为无效
        if(response_header.cmd != LIDAR_CMD_GET_DEVICE_INFO)
        {
            return RESULT_FAIL;
        }
        //接剩下来的字节
        result_t ans = WaitResponseData(reinterpret_cast<uint8_t *>(&info),sizeof(info),timeout);
        if(IS_OK(ans))
        {
            return RESULT_OK;
        }
    }

    return RESULT_FAIL;
}

//复位雷达
result_t LidarSerialDriver::LidarReset(void)
{
    result_t ans;

    //停止点云图数据抓取
    disableDataGrabbing();
    //发送命令
    ScopedLocker l(_lock);      //关闭锁
    if ((ans = SendCommand(LIDAR_CMD_GET_DEVICE_INFO)) != RESULT_OK)
    {
        return ans;
    }
    return RESULT_FAIL;
}

//设置雷达转速信息
result_t LidarSerialDriver::SetScanMotorSpeed(uint16_t frequency, uint16_t &ret_frequency,
                                         uint32_t timeout)
{
    result_t  ans;

    //串口有没有开
    if (!lidar_state.m_SerialOpen)
    {
        return RESULT_FAIL;
    }

    //停止点云图数据抓取
    disableDataGrabbing();
    //刷新串口 并做接收处理乖
    serialport->flush();
    {
        ScopedLocker l(_lock);      //关闭锁
        //发送命令
        if ((ans = SendCommand(LIDAR_CMD_SET_AIMSPEED,
                                    ((uint8_t*)(&frequency)),sizeof(frequency))) != RESULT_OK)
        {
            return ans;
        }

        lidar_ans_header response_header;

        //接收包头
        if ((ans = WaitResponseHeader(&response_header, timeout)) != RESULT_OK)
        {
            return ans;
        }

        //长度比包头还小 认定为不合理
        if (response_header.length < sizeof(lidar_ans_header))
        {
            return RESULT_FAIL;
        }
        //命令字对不上  认为无效
        if(response_header.cmd != LIDAR_CMD_SET_AIMSPEED)
        {
            return RESULT_FAIL;
        }
        //接剩下来的字节
        result_t ans = WaitResponseData(reinterpret_cast<uint8_t *>(&ret_frequency),sizeof(ret_frequency),timeout);

        if(IS_OK(ans))
        {
            lidar_cfg.aimSpeed = frequency;      //采样频率
            return RESULT_OK;
        }
    }

    return RESULT_FAIL;
}

//增加雷达采样率
result_t LidarSerialDriver::SetSamplingRate(uint32_t rate_write, uint32_t &rate,
                                 uint32_t timeout)   //雷达采样率增加
{
    result_t  ans;

    //串口有没有开
    if (!lidar_state.m_SerialOpen)
    {
        return RESULT_FAIL;
    }

    //停止点云图数据抓取
    disableDataGrabbing();
    //刷新串口 并做接收处理
    serialport->flush();
    {
        ScopedLocker l(_lock);      //关闭锁
        //发送命令
        if ((ans = SendCommand(LIDAR_CMD_SET_SAMPLING_RATE,
                                    ((uint8_t*)(&rate_write)),sizeof(rate_write))) != RESULT_OK)
        {
            return ans;
        }

        lidar_ans_header response_header;

        //接收包头
        if ((ans = WaitResponseHeader(&response_header, timeout)) != RESULT_OK)
        {
            return ans;
        }

        //长度比包头还小 认定为不合理
        if (response_header.length < sizeof(lidar_ans_header))
        {
            return RESULT_FAIL;
        }
        //命令字对不上  认为无效
        if(response_header.cmd != LIDAR_CMD_SET_SAMPLING_RATE)
        {
            return RESULT_FAIL;
        }
        //接剩下来的字节
        result_t ans = WaitResponseData(reinterpret_cast<uint8_t *>(&rate),sizeof(rate),timeout);

        if(IS_OK(ans))
        {
            lidar_cfg.samplingRate = rate;      //采样频率
            m_pointTime = 1e9 / lidar_cfg.samplingRate;    //2点之间的时间
            return RESULT_OK;
        }
    }

    return RESULT_FAIL;
}

//读取角度偏移
result_t LidarSerialDriver::GetZeroOffsetAngle(int16_t &angle,uint32_t timeout)
{
    result_t  ans;

    //串口有没有开
    if (!lidar_state.m_SerialOpen)
    {
        return RESULT_FAIL;
    }

    //停止点云图数据抓取
    disableDataGrabbing();
    //刷新串口 并做接收处理
    serialport->flush();
    {
        ScopedLocker l(_lock);      //关闭锁
        //发送命令
        if((ans = SendCommand(LIDAR_CMD_GET_ANGLE_OFFSET)) != RESULT_OK)
        {
            return ans;
        }

        lidar_ans_header response_header;

        //接收包头
        if ((ans = WaitResponseHeader(&response_header, timeout)) != RESULT_OK)
        {
            return ans;
        }

        //长度比包头还小 认定为不合理
        if (response_header.length < sizeof(lidar_ans_header))
        {
            return RESULT_FAIL;
        }
        //命令字对不上  认为无效
        if(response_header.cmd != LIDAR_CMD_GET_ANGLE_OFFSET)
        {
            return RESULT_FAIL;
        }
        //接剩下来的字节
        result_t ans = WaitResponseData(reinterpret_cast<uint8_t *>(&angle),sizeof(angle),timeout);

        if(IS_OK(ans))
        {
            lidar_cfg.angleOffset = angle;      //采样频率
            return RESULT_OK;
        }
    }

    return RESULT_FAIL;
}

//设置角度偏移
result_t LidarSerialDriver::SetZeroOffsetAngle(int16_t angle_set,int16_t &angle,
                                        uint32_t timeout)
{
    result_t  ans;

    //串口有没有开
    if (!lidar_state.m_SerialOpen)
    {
        return RESULT_FAIL;
    }

    //停止点云图数据抓取
    disableDataGrabbing();
    //刷新串口 并做接收处理
    serialport->flush();
    {
        ScopedLocker l(_lock);      //关闭锁
        //发送命令
        if ((ans = SendCommand(LIDAR_CMD_SET_ANGLE_OFFSET,
                                    ((uint8_t*)(&angle_set)),sizeof(angle_set))) != RESULT_OK)
        {
            return ans;
        }

        lidar_ans_header response_header;

        //接收包头
        if ((ans = WaitResponseHeader(&response_header, timeout)) != RESULT_OK)
        {
            return ans;
        }

        //长度比包头还小 认定为不合理
        if (response_header.length < sizeof(lidar_ans_header))
        {
            return RESULT_FAIL;
        }
        //命令字对不上  认为无效
        if(response_header.cmd != LIDAR_CMD_SET_ANGLE_OFFSET)
        {
            return RESULT_FAIL;
        }
        //接剩下来的字节
        result_t ans = WaitResponseData(reinterpret_cast<uint8_t *>(&angle_set),sizeof(angle_set),timeout);

        if(IS_OK(ans))
        {
            lidar_cfg.angleOffset = angle_set;      //采样频率
            angle = angle_set;
            return RESULT_OK;
        }
    }

    return RESULT_FAIL;
}

//设置拖尾等级
result_t LidarSerialDriver::SetTrailingLevel(uint8_t tailing_set, uint8_t &tailing,
                                           uint32_t  timeout)
{
    result_t  ans;

    //串口有没有开
    if (!lidar_state.m_SerialOpen)
    {
        return RESULT_FAIL;
    }

    //停止点云图数据抓取
    disableDataGrabbing();
    //刷新串口 并做接收处理
    serialport->flush();
    {
        ScopedLocker l(_lock);      //关闭锁
        //发送命令
        if((ans = SendCommand(LIDAR_CMD_SET_TAILING_LEVEL,(uint8_t *)(&tailing_set),sizeof(tailing_set)))
                != RESULT_OK)
        {
            return ans;
        }

        lidar_ans_header response_header;

        //接收包头
        if ((ans = WaitResponseHeader(&response_header, timeout)) != RESULT_OK)
        {
            return ans;
        }

        //长度比包头还小 认定为不合理
        if (response_header.length < sizeof(lidar_ans_header))
        {
            return RESULT_FAIL;
        }
        //命令字对不上  认为无效
        if(response_header.cmd != LIDAR_CMD_GET_ANGLE_OFFSET)
        {
            return RESULT_FAIL;
        }
        //接剩下来的字节
        result_t ans = WaitResponseData(reinterpret_cast<uint8_t *>(&tailing),sizeof(tailing),timeout);

        if(IS_OK(ans))
        {
            lidar_cfg.trailingLevel = tailing;      //拖尾等级
            return RESULT_OK;
        }
    }

    return RESULT_FAIL;
}

//获取雷达配置信息
result_t LidarSerialDriver::GetLidarCfg(lidar_get_info &info, uint32_t timeout)
{
    result_t  ans;

    //串口有没有开
    if (!lidar_state.m_SerialOpen)
    {
        return RESULT_FAIL;
    }

    //停止点云图数据抓取
    disableDataGrabbing();
    //刷新串口 并做接收处理
    serialport->flush();
    {
        ScopedLocker l(_lock);      //关闭锁
        //发送命令
        if ((ans = SendCommand(LIDAR_CMD_GET_LIDAR_CFG)) != RESULT_OK)
        {
            return ans;
        }

        lidar_ans_header response_header;

        //接收包头
        if ((ans = WaitResponseHeader(&response_header, timeout)) != RESULT_OK)
        {
            return ans;
        }

        //长度比包头还小 认定为不合理
        if (response_header.length < sizeof(lidar_ans_header))
        {
            return RESULT_FAIL;
        }
        //命令字对不上  认为无效
        if(response_header.cmd != LIDAR_CMD_GET_LIDAR_CFG)
        {
            return RESULT_FAIL;
        }

        //接剩下来的字节
        result_t ans = WaitResponseData(reinterpret_cast<uint8_t *>(&info),sizeof(lidar_get_info),timeout);

        if(IS_OK(ans))
        {
            lidar_info = info;

            m_pointTime = 1e9 / lidar_info.samplingRate;    //采样频率
            lidar_cfg.aimSpeed = lidar_info.aimSpeed;
            lidar_cfg.samplingRate = lidar_info.samplingRate;
            lidar_cfg.trailingLevel = lidar_info.trailLevel;
            lidar_cfg.IsHasSensitive = lidar_info.hasSensitive;

            return RESULT_OK;
        }
    }
    return RESULT_FAIL;
}

//保存雷达参数
result_t LidarSerialDriver::SaveCfg(bool &flag, uint32_t timeout)
{
    result_t  ans;
    uint8_t   save_flag = 0;    //保存标记信息

    //串口有没有开
    if (!lidar_state.m_SerialOpen)
    {
        return RESULT_FAIL;
    }

    //停止点云图数据抓取
    disableDataGrabbing();
    //刷新串口 并做接收处理
    serialport->flush();
    {
        ScopedLocker l(_lock);      //关闭锁
        //发送命令
        if ((ans = SendCommand(LIDAR_CMD_SAVE_LIDAR_PARA)) != RESULT_OK)
        {
            return ans;
        }

        lidar_ans_header response_header;

        //接收包头
        if ((ans = WaitResponseHeader(&response_header, timeout)) != RESULT_OK)
        {
            return ans;
        }

        //长度比包头还小 认定为不合理
        if (response_header.length < sizeof(lidar_ans_header))
        {
            return RESULT_FAIL;
        }
        //命令字对不上  认为无效
        if(response_header.cmd != LIDAR_CMD_SET_ANGLE_OFFSET)
        {
            return RESULT_FAIL;
        }
        //接剩下来的字节
        result_t ans = WaitResponseData(reinterpret_cast<uint8_t *>(&save_flag),sizeof(save_flag),timeout);

        if(IS_OK(ans))
        {
            //是否保存成功
            if(save_flag)
            {
                flag = save_flag;
                return RESULT_OK;
            }
        }
    }
    return RESULT_FAIL;
}

//雷达等待点云图的接收数据信息
result_t LidarSerialDriver::WaitPointSinglePackData(std::vector<node_info> &info,uint32_t timeout)
{
    uint32_t    startTs = getms();           //获取当前ms数
    bool        package_timeout_falg = true;    //超时标记
    bool        package_crc_error = true;      //校验错误
    int         recvPos = 0;                    //当前接到的位置信息
    uint32_t    waitTime;
    size_t      recvSize = 0;
    uint8_t     package_point_num = 0;          //当前点云 一包点数据
    uint16_t    package_first_angle_temp = 0;   //起始角
    uint16_t    package_last_angle_temp = 0;    //结束角
    std::vector<node_info> package_pointlist;   //点云信息
    static uint32_t     package_after_0c = 0;   //0度后的第几包
    uint16_t    package_speed = 0;              //转速信息
    uint16_t    package_temp = 0;               //温度信息
    uint16_t    checksum_speed_temp = 0;        //校验计算
    uint16_t    checksum_packnum_index = 0;     //包数目和0位索引校验
    uint16_t    checksum_temp = 0;              //临时校验计算
    uint64_t    package_starttime = 0;          //起始时间戳 

    //先将包数据清掉
    memset(packageInfo.packageBuffer.buf,0x00,sizeof(packageInfo.packageBuffer.buf));

    //get起始时间戳 
    package_starttime = getTime();

    //如果未接到包头部分  则开始先接包头的信息
    //-----------------------接包头数据 10byte-----------------------
    package_timeout_falg = true;        //超时标记置有效
    while ((waitTime = getms() - startTs) <= timeout/2)
    {
        //读字节剩余长度信息
        result_t ans = RecvLenSerial(&recvSize);  //读缓存内已有字节长度信息
        //超时未接全包头 则认为有问题
        if (!IS_OK(ans))
        {
            return ans;
        }

        //超过部分来读
        if(recvSize >= LIDAR_PACKAGE_HEAD_SIZE)
        {
            recvSize = LIDAR_PACKAGE_HEAD_SIZE;

            //读数据
            ans = RecvSerial(packageInfo.packageBuffer.buf,recvSize);
            if (!IS_OK(ans))
            {
                return ans;
            }

            //包头数据解析
            for(size_t i = 0; i<recvSize; i++)
            {
                uint8_t cur_byte = packageInfo.packageBuffer.buf[i];     //取到当前的字节数

                switch (recvPos)
                {
                    case 0:     //第一个字节 包头
                    {
                        if(cur_byte == (uint8_t)(LIDAR_POINT_HEADER & 0xFF))
                        {
                            recvPos++;      //index后移
                            //printf("get first head\n");
                        }
                        else        //没收到 直接发下一包
                        {
                            continue;
                        }
                        break;
                    }
                    case 1:     //第二字节  包头信息
                    {
                        if(cur_byte == (uint8_t)(LIDAR_POINT_HEADER >> 8))
                        {
                            packageInfo.packageCheckSumCalc = LIDAR_POINT_HEADER; //更新校验值
                            recvPos++;      //index后移
                            //printf("get second head\n");
                        }
                        else
                        {
                            packageInfo.packageErrFlag = true;      //包头错误鸟
                            recvPos = 0;
                            continue;
                        }
                        break;
                    }
                    case 2:     //频率或温度等信息
                    {
                        checksum_speed_temp = cur_byte;     //校验赋值

                        //0度角或其它信息
                        if(1 == package_after_0c)  //其它  0位后第1包为  温度值
                        {
                            packageInfo.packageHas0CFirst = false;
                            packageInfo.packageHasTempFirst = true;

                            package_temp = cur_byte;
                        }
                        else if(cur_byte & 0x01)     //最低位是0位
                        {
                            packageInfo.packageHas0CFirst = true;
                            packageInfo.packageHasTempFirst = false;

                            package_speed = cur_byte;
                        }
                        else        //其它情况  该位置不含其它信息
                        {
                            packageInfo.packageHas0CFirst = false;
                            packageInfo.packageHasTempFirst = false;
                        }
                        recvPos++;      //index后移
                        break;
                    }
                    case 3:         //频率或者温度
                    {
                        checksum_speed_temp += (cur_byte*256);     //校验计算
                        packageInfo.packageCheckSumCalc ^= checksum_speed_temp; //校验计算

                        if(packageInfo.packageHas0CFirst)  //可能有0度
                        {
                             packageInfo.packageHas0CFirst = false;

                             packageInfo.packageHasTemp = false;

                             if(cur_byte & 0x80)
                              {
                                  package_after_0c = 0;     //0位包  则将0度后的个数  清0

                                  packageInfo.packageHas0CAngle = true;
                                  package_speed += ((uint16_t)cur_byte *256);
                                  packageInfo.packageFreq = (package_speed & 0x7FFF) >> 1;
                              }
                              else
                              {
                                  packageInfo.packageHas0CAngle = false;
                              }
                        }
                        else if(packageInfo.packageHasTempFirst)    //是温度计算信息
                        {
                            packageInfo.packageHasTempFirst = false;

                            packageInfo.packageHas0CAngle = false;


                            packageInfo.packageHasTemp = true;
                            package_temp += ((uint16_t)cur_byte *256);
                            packageInfo.packageTemp = (int16_t)(package_temp);
                        }
                        else
                        {
                            packageInfo.packageHas0CAngle = false;
                            packageInfo.packageHasTemp = false;
                        }
                        package_after_0c++;     //0度后的包数目

                        recvPos++;      //index后移
                        break;
                    }
                    case 4:     //包数目
                    {
                        checksum_packnum_index = cur_byte;

                        if (cur_byte != 0)
                        {
                            package_point_num = cur_byte;
                            recvPos++;      //index后移
                        }
                        else
                        {
                            packageInfo.packageErrFlag = true;      //包头错误鸟
                            recvPos = 0;
                            continue;
                        }
                        //printf("package num:%d\r\n",package_point_num);
                        break;
                    }
                    case 5:     //0度索引
                    {
                        checksum_packnum_index += (uint16_t)cur_byte*256;
                        packageInfo.packageCheckSumCalc ^= checksum_packnum_index; //校验计算

                        if(packageInfo.packageHas0CAngle)       //如果是0c  则会告知0c index
                        {
                            if(cur_byte > 0)
                            {
                                packageInfo.package0CIndex = cur_byte - 1;      //0度角
                            }
                            //printf("0c index:%d\r\n",packageInfo.package0CIndex);
                        }
                        recvPos++;
                        break;
                    }
                    case 6:             //起始角度低位
                    {
                        if (cur_byte & LIDAR_RESP_MEASUREMENT_CHECKBIT)
                        {
                            package_first_angle_temp = cur_byte;
                            recvPos++;      //index后移
                        }
                        else
                        {
                            packageInfo.packageErrFlag = true;
                            recvPos = 0;
                            continue;
                        }
                        break;
                    }
                    case 7:             //起始角度高位
                    {
                        package_first_angle_temp += (uint16_t)cur_byte * 256;
                        packageInfo.packageCheckSumCalc ^= package_first_angle_temp;
                        packageInfo.packageFirstAngle = package_first_angle_temp >> 1;

                        //printf("first angle = %f\n",(float)packageInfo.packageFirstAngle/64.0f);

                        recvPos++;      //index后移
                        break;
                    }
                    case 8:             //结束角低位
                    {
                        if (cur_byte & LIDAR_RESP_MEASUREMENT_CHECKBIT)
                        {
                            package_last_angle_temp = cur_byte;

                          //  printf("last_angle_l = %d\n",package_last_angle_temp);

                            recvPos++;      //index后移
                        }
                        else
                        {
                            packageInfo.packageErrFlag = true;
                            recvPos = 0;
                            continue;
                        }
                        break;
                    }
                    case 9:             //结束角高位
                    {
                        package_last_angle_temp += (uint16_t)cur_byte * 0x100;
                        packageInfo.packageCheckSumCalc ^= package_last_angle_temp;
                        packageInfo.packageLastAngle = package_last_angle_temp >> 1;

                        //printf("last angle = %f\n",(float)packageInfo.packageLastAngle/64.0f);

                        //计算每个角度之间的差值信息
                        if(1 == package_point_num)  //只有一个点  则没有差值
                        {
                            packageInfo.packageAngleDiffer = 0;
                        }
                        else
                        {
                            //结束角小于起始角
                            if (packageInfo.packageLastAngle < packageInfo.packageFirstAngle)
                            {
                                //270~90度
                                if ((packageInfo.packageFirstAngle > 270 * LIDAR_ANGULDAR_RESOLUTION) && (packageInfo.packageLastAngle < 90 * LIDAR_ANGULDAR_RESOLUTION))
                                {
                                  packageInfo.packageAngleDiffer =
                                     (float)((float)(360 * LIDAR_ANGULDAR_RESOLUTION + packageInfo.packageLastAngle - packageInfo.packageFirstAngle) /
                                             ((float)(package_point_num - 1)));
                                  packageInfo.packageLastAngleDiffer = packageInfo.packageAngleDiffer;
                                }
                                else
                                {
                                   packageInfo.packageAngleDiffer = packageInfo.packageLastAngleDiffer;
                                }
                            }
                            //结束角大于等于起始角
                            else
                            {
                                packageInfo.packageAngleDiffer =
                                        (float)((float)(packageInfo.packageLastAngle - packageInfo.packageFirstAngle) /
                                            (float)(package_point_num - 1));
                                packageInfo.packageLastAngleDiffer = packageInfo.packageAngleDiffer;
                            }
                        }

                        recvPos++;      //index后移

                        break;
                    }
                    case 10:    //校验低位
                    {
                        packageInfo.packageCheckSumGet = cur_byte;

                        recvPos++;      //index后移
                        break;
                    }
                    case 11:     //校验高位
                    {
                        packageInfo.packageCheckSumGet += cur_byte * 256;

                        recvPos++;      //index后移

                        break;
                    }
                }
            }
            if(recvPos >= LIDAR_PACKAGE_HEAD_SIZE)
            {
                package_timeout_falg = false;
                break;      //退出超时等待
            }
        }
    }
    //超时
    if(package_timeout_falg)       //超时了 有效包头还没接满 则直接退出
    {
        return RESULT_TIMEOUT;
    }


    //-------------------------接包头后距离数据-------------------------
    startTs = getms();           //获取当前ms数
    size_t      remain_size = 0;
    if(lidar_cfg.IsHasSensitive)
    {
        packageInfo.packagePointDistSize = 4;
    }
    else
    {
        packageInfo.packagePointDistSize = 2;
    }
    remain_size = package_point_num * packageInfo.packagePointDistSize; //剩余的距离数据信息

    package_timeout_falg = true;
    while ((waitTime = getms() - startTs) <= timeout/2)
    {
        //读字节剩余长度信息
        result_t ans = RecvLenSerial(&recvSize);  //读缓存内已有字节长度信息
        //超时未接全包头 则认为有问题
        if (!IS_OK(ans))
        {
            return ans;
        }

        //超过部分来读
        if(recvSize >= remain_size)
        {
            recvSize = remain_size;
            m_packageTime = getTime() - package_starttime;        //结束时间减去起始时间 

            //数据解析处理
            ans = RecvSerial(packageInfo.packageBuffer.buf + LIDAR_PACKAGE_HEAD_SIZE,recvSize);
            if (!IS_OK(ans))
            {
                return ans;
            }

            package_timeout_falg = false;       //超时标记

            //计算校验
            for(size_t j = 0; j<recvSize; j++)
            {
                if(j % 2 == 0)
                {
                    checksum_temp = packageInfo.packageBuffer.buf[LIDAR_PACKAGE_HEAD_SIZE + j];  //低位
                }
                else
                {
                    checksum_temp += (uint16_t)(packageInfo.packageBuffer.buf[LIDAR_PACKAGE_HEAD_SIZE + j])*256;
                    packageInfo.packageCheckSumCalc ^= checksum_temp;
                }
            }
            if(packageInfo.packageCheckSumCalc == packageInfo.packageCheckSumGet)
            {
                package_crc_error = false;      //校验错误 清掉
            }

            break;
        }
    }
    //校验错误
    if(package_crc_error)
    {
        return RESULT_FAIL;
    }
    //超时
    if(package_timeout_falg)       //超时了 字节还没接够  则返回超时状态信息
    {
        return RESULT_TIMEOUT;
    }

    //----------------计算角度及距离信息 list-------------------------
    package_pointlist.clear();

    for(int i = 0; i<package_point_num; i++)
    {
        node_info node;

        if(lidar_cfg.IsHasSensitive)
        {
            //点信息更新
            node.lidar_distance = packageInfo.packageBuffer.pack_qua.packageSample[i].PakageSampleDistance;     //距离
            node.lidar_quality = packageInfo.packageBuffer.pack_qua.packageSample[i].PakageSampleQuality;       //信号质量
            node.lidar_speed = (float)(packageInfo.packageFreq)/100.0;  //转速
            node.lidar_temper = (float)(packageInfo.packageTemp)/10.0;      //温度
            node.lidar_point_time = packageInfo.packagePointTime;           //采样率
            node.index = i;                 //当前索引
            //是0度角
            if(packageInfo.packageHas0CAngle)
            {
                if(i == packageInfo.package0CIndex)
                {
                    node.lidar_angle_zero_flag = true;
                }
                else
                {
                    node.lidar_angle_zero_flag = false;
                }
            }
            else
            {
                node.lidar_angle_zero_flag = false;
            }

            //角度计算
            node.lidar_angle = (float)(packageInfo.packageFirstAngle + i*packageInfo.packageAngleDiffer)
                                    /(float)(LIDAR_ANGULDAR_RESOLUTION);
            if(node.lidar_angle >= 360.0)
            {
                node.lidar_angle -= 360.0;
            }
        }
        else
        {
            //点信息更新
            node.lidar_distance = packageInfo.packageBuffer.pack_no_qua.packageSample[i].PakageSampleDistance;     //距离
            node.lidar_quality = 0;
            node.lidar_speed = (float)(packageInfo.packageFreq)/100.0;  //转速
            node.lidar_temper = (float)(packageInfo.packageTemp)/10.0;      //温度
            node.lidar_point_time = packageInfo.packagePointTime;           //采样率
            node.index = i;                 //当前索引
            //是0度角
            if(packageInfo.packageHas0CAngle)
            {
                if(i == packageInfo.package0CIndex)
                {
                    m_0cIndex = packageInfo.package0CIndex; //求的0度所在位置  
                    node.lidar_angle_zero_flag = true;
                }
                else
                {
                    node.lidar_angle_zero_flag = false;
                }
            }
            else
            {
                node.lidar_angle_zero_flag = false;
            }

            //角度计算
            node.lidar_angle = (float)(packageInfo.packageFirstAngle + i*packageInfo.packageAngleDiffer)
                                    /(float)(LIDAR_ANGULDAR_RESOLUTION);
            if(node.lidar_angle >= 360.0)
            {
                node.lidar_angle -= 360.0;
            }
        }

        package_pointlist.push_back(node);  //追加到列表内
    }

    if(packageInfo.packageCheckSumCalc == packageInfo.packageCheckSumGet)
    {
        info = package_pointlist;       //容器赋值
    }
    else 
    {
        //printf("crc error cal:%04x,get:%04x\r\n",packageInfo.packageCheckSumCalc,packageInfo.packageCheckSumGet);
        info.clear();
    }

    return RESULT_OK;       //成功
}

//雷达扫描一圈点
result_t LidarSerialDriver::WaitPointCircleData()
{
    result_t ans = RESULT_GETTING;
    std::vector<node_info> package_pack_temp;  //单独一包的点云信息
    static std::vector<node_info> circle_point;        //一圈的点云数据

    //取得单点的数据值信息
    result_t p_ans = WaitPointSinglePackData(package_pack_temp);

    if(p_ans == RESULT_OK)
    {
        //print thread
        //printf("thread_ok\r\n");
        //轮询
        for(size_t i = 0; i< package_pack_temp.size(); i++)
        {
            if(package_pack_temp.at(i).lidar_angle_zero_flag == true)  //检测到0度角了 则为下一包的数据
            {
                m_first_circle_finish = true;

                if(true == m_first_circle_finish)
                {
                    _lock.lock();   //加锁

                    //printf("pack finish = %d\r\n",circle_point.size());
                    circle_node_points = circle_point;                      //一圈点数据
                    circle_point.clear();                                   //清除掉  缓存下一包点的数据信

                    _dataEvent.set();
                    _lock.unlock(); //解锁
                }
                else
                {
                    circle_point.clear();
                    m_first_circle_finish = true;
                }
            }
            circle_point.push_back(package_pack_temp.at(i));            //添加数据进来
        }
    }
    return ans;                         //返回当前状态信息
}

//雷达扫描线程
int LidarSerialDriver::ScanThread()
{
   // printf("thread run\r\n");
    //如果正在出图
    while(lidar_state.m_Scanning)
    {
        WaitPointCircleData();
    }

    return 0;
}


//禁止抓点云接口
void LidarSerialDriver::disableDataGrabbing()
{
    if (lidar_state.m_Scanning)
    {
        lidar_state.m_Scanning = false;
        _dataEvent.set();
    }
    _thread.join();
}

//抓取点云数据接口
result_t LidarSerialDriver::grabScanData(std::vector<node_info>&info,uint32_t timeout)
{
    switch (_dataEvent.wait(timeout))
    {
        case Event::EVENT_TIMEOUT:
        {
            //printf("timeout\r\n");
            circle_node_points.clear();
            info.clear();       //超时
            return RESULT_TIMEOUT;
        }
        case Event::EVENT_OK:
        {
            if(0 == circle_node_points.size())
            {
                return RESULT_FAIL;
            }
            ScopedLocker l(_lock);
            info = circle_node_points;

            // for(int i = 0; i<info.size(); i++)
            // {
            //     if(info.at(i).lidar_distance > 60000)
            //     {
            //         printf("error_dis:%d\r\n",info.at(i).lidar_distance);
            //     }
            // }
            return RESULT_OK;
        }
        default:
        {
            info.clear();
            break;
        }
    }
}

//启动雷达
result_t LidarSerialDriver::StartScan()
{
    result_t ans;
    //串口有没有开
    if (!lidar_state.m_SerialOpen)
    {
        return RESULT_FAIL;
    }
    //是否正在运行
    if(lidar_state.m_Scanning)
    {
        return RESULT_OK;
    }

   serialport->flush();        //刷新串口

   //first circle false
   m_first_circle_finish = false;

    //发送数据
    if ((ans = SendCommand(LIDAR_CMD_SCAN)) != RESULT_OK) {
      return ans;
    }
    delay(10);
    if ((ans = SendCommand(LIDAR_CMD_SCAN)) != RESULT_OK) {
      return ans;
    }

    //创建线程 此线程用于扫描出图等
    ans = createThread();
    if(!IS_OK(ans))
    {
        return ans;
    }

    return RESULT_OK;
}

//雷达停止
result_t  LidarSerialDriver::StopScan()
{
    void *res;
    int s = 0;

    //if(isAutoconnting)
    {
     // isAutoReconnect = false;
     // lidar_state.m_Scanning = false;
     // disableDataGrabbing();
     // return RESULT_OK;
    }

    LidarStopCmd();             //发送停止命令

   // printf("stop scan id\r\n");

    //停止线程 如果有线程开启
    _thread.join();
    //扫描标记清0
    lidar_state.m_Scanning = false;
    return RESULT_OK;
}

//雷达发送停止命令
result_t  LidarSerialDriver::LidarStopCmd()
{
    //发送数据
    SendCommand(LIDAR_CMD_STOP);
    delay(10);
    //发送数据
    SendCommand(LIDAR_CMD_STOP);
    delay(10);

    return RESULT_OK;
}

//获取当前包传送的时间
uint32_t LidarSerialDriver::getPackageTime()
{
    return m_packageTime;
}
//获取当前扫描状态
uint32_t LidarSerialDriver::getScanState()
{
    return lidar_state.m_Scanning;
}
//获取0位包  已传输时间
uint32_t LidarSerialDriver::getZeroIndex()
{
    return m_0cIndex;
}

}//namespace


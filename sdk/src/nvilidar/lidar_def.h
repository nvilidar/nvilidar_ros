#ifndef _LIDAR_DEF_H_
#define _LIDAR_DEF_H_

#include <stdint.h>
#include "lidar_protocol.h"

//超时时间等信息
typedef int32_t result_t;

#define RESULT_OK           0
#define RESULT_TIMEOUT      -1
#define RESULT_FAIL         -2
#define RESULT_GETTING      -3

//其它
#define DEFAULT_TIMEOUT     2000    //默认超时时间

#define IS_OK(x)       ( (x) == RESULT_OK )
#define IS_TIMEOUT(x)  ( (x) == RESULT_TIMEOUT )
#define IS_FAIL(x)     ( (x) == RESULT_FAIL )
#define IS_GET(x)      ( (x) == RESULT_GETTING )

#define SDKVerision     "1.0.1"

//雷达型号
enum
{
   NVILIDAR_VP300      = 1,
   NVILIDAR_Tail,
};


#endif

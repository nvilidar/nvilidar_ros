#ifndef _TIMER_H_
#define _TIMER_H_

#include <assert.h>
#include <time.h>
#include <inttypes.h>


#define BEGIN_STATIC_CODE( _blockname_ ) \
    static class _static_code_##_blockname_ {   \
    public:     \
    _static_code_##_blockname_ ()


#define END_STATIC_CODE( _blockname_ ) \
    }   _instance_##_blockname_;


//延时等待sleep
#if defined(_WIN32)
#include <windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <sys/time.h>
#include <unistd.h>

static inline void delay(uint32_t ms) {
  while (ms >= 1000) {
    usleep(1000 * 1000);
    ms -= 1000;
  };

  if (ms != 0) {
    usleep(ms * 1000);
  }
}
#endif

//函数声明
#if defined(_WIN32)
    void HPtimer_reset();
#endif
    uint32_t getHDTimer();
    uint64_t getCurrentTime();


//获取时间戳信息
#define getms()     getHDTimer()              //获取ms值
#define getTime()   getCurrentTime()          //获取当前Ns值

#endif

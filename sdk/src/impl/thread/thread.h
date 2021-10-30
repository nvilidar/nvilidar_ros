#pragma once

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#include <io.h>
#include <process.h>
#include <stdint.h>
#else
#include <pthread.h>
#include <assert.h>
#include <stdio.h>
#endif

#ifndef __GNUC__
#define __attribute__(x)
#endif

#define __small_endian

//PROC定义
#ifdef _AVR_
    typedef uint8_t        _size_t;
    #define THREAD_PROC
#elif defined (WIN64)
    typedef uint64_t       _size_t;
    #define THREAD_PROC    __stdcall
#elif defined (WIN32)
    typedef uint32_t       _size_t;
    #define THREAD_PROC    __stdcall
#elif defined (_M_X64)
    typedef uint64_t       _size_t;
    #define THREAD_PROC    __stdcall
#elif defined (__GNUC__)
    typedef unsigned long  _size_t;
    #define THREAD_PROC
#elif defined (__ICCARM__)
    typedef uint32_t       _size_t;
    #define THREAD_PROC
#endif

//宏定义
#define UNUSED(x) (void)x
typedef _size_t (THREAD_PROC *thread_proc_t)(void *);

//宏定义 初始化 
#define CLASS_THREAD(c , x ) 		Thread::ThreadCreateObjectFunctor<c, &c::x>(this)

//WIN系统  
#if defined(_WIN32)

class Thread 
{
	public:
		//创建线程 
		template <class CLASS, int (CLASS::*PROC)(void)> static Thread
		ThreadCreateObjectFunctor(CLASS *pthis) 
		{
			return createThread(createThreadAux<CLASS, PROC>, pthis);
		}

		template <class CLASS, int (CLASS::*PROC)(void) > static _size_t THREAD_PROC
		createThreadAux(void *param) 
		{
			return (static_cast<CLASS *>(param)->*PROC)();
		}
		
		//创建线程 
        static Thread createThread(thread_proc_t proc, void *param = NULL)
		{
			Thread thread_(proc, param);
			thread_._handle = (_size_t)(_beginthreadex(NULL, 0,
                                    (unsigned int (__stdcall *)(void *))proc, param, 0, NULL));
			return thread_;
		}

	public:
		explicit Thread(): _param(NULL), _func(NULL), _handle(0) {}
		virtual ~Thread() {}
		//获取线程ID 
		_size_t getHandle() 
		{
			return _handle;
		}	
		//中止 
		int terminate() 
		{
			if (!this->_handle) 
			{
				return 0;
			}

			if (TerminateThread(reinterpret_cast<HANDLE>(this->_handle), -1)) 
			{
				CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
				this->_handle = NULL;
				return 0;
			} 
			else 
			{
				return -2;
			}
		}
		//等待线程结束  
		int join(unsigned long timeout = -1) 
		{
			if (!this->_handle) 
			{
			  return 0;
			}
			
			switch (WaitForSingleObject(reinterpret_cast<HANDLE>(this->_handle), timeout)) 
			{
				case WAIT_OBJECT_0:
				{
					CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
					this->_handle = NULL;
					return 0;
				}
				case WAIT_ABANDONED:
				{
					return -2;
				}
				case WAIT_TIMEOUT:
				{
					return -1;
				}
			}
            return 0;
		}
		
		//判断线程是否相等
		bool operator== (const Thread &right) 
		{
			return this->_handle == right._handle;
		}
		
	 protected:
		explicit Thread(thread_proc_t proc, void *param): _param(param), _func(proc),
		_handle(0) {}
		void *_param;
		thread_proc_t _func;
		_size_t _handle;
};


#else 
    class Thread
    {
        public:
            //创建线程
            template <class CLASS, int (CLASS::*PROC)(void)> static Thread
            ThreadCreateObjectFunctor(CLASS *pthis)
            {
                return createThread(createThreadAux<CLASS, PROC>, pthis);
            }

            template <class CLASS, int (CLASS::*PROC)(void) > static _size_t THREAD_PROC
            createThreadAux(void *param)
            {
                return (static_cast<CLASS *>(param)->*PROC)();
            }

            //创建线程
            static Thread createThread(thread_proc_t proc, void *param = NULL)
            {
                Thread thread_(proc, param);
                assert(sizeof(thread_._handle) >= sizeof(pthread_t));
                pthread_create((pthread_t *)&thread_._handle, NULL, (void *(*)(void *))proc,
                                   param);
                return thread_;
            }

        public:
            explicit Thread(): _param(NULL), _func(NULL), _handle(0) {}
            virtual ~Thread() {}
            //获取线程ID
            _size_t getHandle()
            {
                return _handle;
            }
            //中止
            int terminate()
            {
                if (!this->_handle)
                {
                    return 0;
                }

                return pthread_cancel((pthread_t)this->_handle);
            }
            //等待线程结束
            int join(unsigned long timeout = -1)
            {
                if (!this->_handle)
                {
                  return 0;
                }

                UNUSED(timeout);
                void *res;
                int s;
                s = pthread_cancel((pthread_t)(this->_handle));
                if (s != 0) {}

                s = pthread_join((pthread_t)(this->_handle), &res);
                if (s != 0) {}

                if (res == PTHREAD_CANCELED)
                {
                  printf("%lu thread has been canceled\n", this->_handle);
                  fflush(stdout);
                  this->_handle = 0;
                }
                return 0;
            }

            //判断线程是否相等
            bool operator== (const Thread &right)
            {
                return this->_handle == right._handle;
            }

         protected:
            explicit Thread(thread_proc_t proc, void *param): _param(param), _func(proc),
            _handle(0) {}
            void *_param;
            thread_proc_t _func;
            _size_t _handle;
    };

#endif 

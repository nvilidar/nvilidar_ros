#pragma once

#if defined(_WIN32)
	#include <conio.h>
	#include <windows.h>
	#include <process.h>
	#include <tlhelp32.h>
	#include <sys/utime.h>
	#include <io.h>
	#include <direct.h>
#else
	#include <assert.h>
	#include <pthread.h>
	#include <sys/time.h>
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <errno.h>
    #include <stdio.h>
    #include <stdlib.h>
#endif

#if defined(_WIN32)

class Locker 
{
public:
	//锁状态 
	enum LOCK_STATUS 
	{
		LOCK_OK = 0,
		LOCK_TIMEOUT = -1,
		LOCK_FAILED = -2
	};
	//构造函数  
	Locker() 
	{
		_lock = NULL;
		init();
	}
	//析构函数  
	~Locker() 
	{
		release();
	}

	//上锁 
	Locker::LOCK_STATUS lock(unsigned long timeout = 0xFFFFFFFF) 
	{
		//等待信息 
		switch (WaitForSingleObject(_lock,
									timeout == 0xFFFFFFF ? INFINITE : (DWORD)timeout)) 
		{
			case WAIT_ABANDONED:
				return LOCK_FAILED;

			case WAIT_OBJECT_0:
				return LOCK_OK;

			case WAIT_TIMEOUT:
				return LOCK_TIMEOUT;
		}
        return LOCK_FAILED;
	}

	//解锁 
	void unlock() 
	{
		ReleaseMutex(_lock);
	}

	//获取锁状态 
	HANDLE getLockHandle() 
	{
		return _lock;
	}


protected:
	//初始化 
	void init() 
	{
		_lock = CreateMutex(NULL, FALSE, NULL);
	}

	//释放 
	void release() 
	{
		unlock();
		if (_lock) 
		{
			CloseHandle(_lock);
		}
		_lock = NULL;
	}
	HANDLE  _lock;
};

//事件 
class Event 
{
 public:
	enum 
	{
		EVENT_OK = 1,
		EVENT_TIMEOUT = 2,
		EVENT_FAILED = 0,
	};

	//构造函数 
	explicit Event(bool isAutoReset = true, bool isSignal = false): _event(NULL)
	{
		_event = CreateEvent(NULL, isAutoReset ? FALSE : TRUE, isSignal ? TRUE : FALSE,
							 NULL);
	}
	//析构函数 
	~ Event() 
	{
		release();
	}

	//使能 
	void set(bool isSignal = true) 
	{
		if (isSignal) 
		{
			SetEvent(_event);
		} 
		else 
		{
			ResetEvent(_event);
		}
	}

	//等待 
	unsigned long wait(unsigned long timeout = 0xFFFFFFFF) 
	{
		switch (WaitForSingleObject(_event,
									timeout == 0xFFFFFFF ? INFINITE : (DWORD)timeout)) 
		{
			case WAIT_FAILED:
				return EVENT_FAILED;

			case WAIT_OBJECT_0:
				return EVENT_OK;

			case WAIT_TIMEOUT:
				return EVENT_TIMEOUT;
		}
		return EVENT_OK;
	}
	protected:

	//释放 
	void release() 
	{
		CloseHandle(_event);
	}
	HANDLE _event;
};


#else 

	class Locker 
	{
	public:
		//锁状态 
		enum LOCK_STATUS 
		{
			LOCK_OK = 0,
			LOCK_TIMEOUT = -1,
			LOCK_FAILED = -2
		};
		//构造函数  
		Locker() 
		{
			init();
		}
		//析构函数  
		~Locker() 
		{
			release();
		}

		//上锁 
		Locker::LOCK_STATUS lock(unsigned long timeout = 0xFFFFFFFF) 
		{
			if (timeout == 0xFFFFFFFF) 
			{
				if (pthread_mutex_lock(&_lock) == 0) 
				{
					return LOCK_OK;
				}
			}
			else if (timeout == 0) 
			{
				if (pthread_mutex_trylock(&_lock) == 0) 
				{
					return LOCK_OK;
				}
			}
			else 
			{
				timespec wait_time;
				timeval now;
				gettimeofday(&now, NULL);

				wait_time.tv_sec = timeout / 1000 + now.tv_sec;
				wait_time.tv_nsec = (timeout % 1000) * 1000000 + now.tv_usec * 1000;

				if (wait_time.tv_nsec >= 1000000000) 
				{
					++wait_time.tv_sec;
					wait_time.tv_nsec -= 1000000000;
				}
				
				switch (pthread_mutex_timedlock(&_lock, &wait_time)) 
				{
					case 0:
					  return LOCK_OK;

					case ETIMEDOUT:
					  return LOCK_TIMEOUT;
				}
			}
			return LOCK_FAILED;
		}

		//解锁 
		void unlock() 
		{
			pthread_mutex_unlock(&_lock);
		}

		//获取锁状态 
		pthread_mutex_t *getLockHandle() 
		{
			return &_lock;
		}


	protected:
		//初始化 
		void init() 
		{
			pthread_mutex_init(&_lock, NULL);
		}

		//释放 
		void release() 
		{
			unlock();
			pthread_mutex_destroy(&_lock);
		}
		pthread_mutex_t _lock;
	};

	//事件 
	class Event 
	{
	 public:
		enum 
		{
			EVENT_OK = 1,
			EVENT_TIMEOUT = 2,
			EVENT_FAILED = 0,
		};

		//构造函数 
		explicit Event(bool isAutoReset = true, bool isSignal = false): _is_signalled(isSignal),_isAutoReset(isAutoReset)
		{
			int ret = pthread_condattr_init(&_cond_cattr);

			if (ret != 0) 
			{
				fprintf(stderr, "Failed to init condattr...\n");
				fflush(stderr);
				exit(1);
			}

			ret = pthread_condattr_setclock(&_cond_cattr, CLOCK_MONOTONIC);
			pthread_mutex_init(&_cond_locker, NULL);
			ret =  pthread_cond_init(&_cond_var, &_cond_cattr);
		}
		//析构函数 
		~ Event() 
		{
			release();
		}

		//使能 
		void set(bool isSignal = true) 
		{
			if (isSignal) 
			{
				pthread_mutex_lock(&_cond_locker);

				if (_is_signalled == false) 
				{
					_is_signalled = true;
					pthread_cond_signal(&_cond_var);
				}
				pthread_mutex_unlock(&_cond_locker);
			} 
			else 
			{
				pthread_mutex_lock(&_cond_locker);
				_is_signalled = false;
				pthread_mutex_unlock(&_cond_locker);
			}
		}

		//等待 
		unsigned long wait(unsigned long timeout = 0xFFFFFFFF) 
		{
			unsigned long ans = EVENT_OK;
			pthread_mutex_lock(&_cond_locker);

			if (!_is_signalled) 
			{
				if (timeout == 0xFFFFFFFF) 
				{
					pthread_cond_wait(&_cond_var, &_cond_locker);
				} 
				else 
				{
					struct timespec wait_time;
					clock_gettime(CLOCK_MONOTONIC, &wait_time);
					wait_time.tv_sec += timeout / 1000;
					wait_time.tv_nsec += (timeout % 1000) * 1000000ULL;

					if (wait_time.tv_nsec >= 1000000000) 
					{
						++wait_time.tv_sec;
						wait_time.tv_nsec -= 1000000000;
					}

					switch (pthread_cond_timedwait(&_cond_var, &_cond_locker, &wait_time)) 
					{
						case 0:
						{
							// signalled
							break;
						}

						case ETIMEDOUT:
						{
							// time up
							ans = EVENT_TIMEOUT;
							goto _final;
							break;
						}

						default:
						{
							ans = EVENT_FAILED;
							goto _final;
						}
					}
				}
			}

			assert(_is_signalled);

			if (_isAutoReset) 
			{
				_is_signalled = false;
			}

			_final:
			{
				pthread_mutex_unlock(&_cond_locker);
			}

			return ans;
		}
		protected:

		//释放 
		void release() 
		{
			pthread_condattr_destroy(&_cond_cattr);
			pthread_mutex_destroy(&_cond_locker);
			pthread_cond_destroy(&_cond_var);
		}
		
		pthread_condattr_t     _cond_cattr;
		pthread_cond_t         _cond_var;
		pthread_mutex_t        _cond_locker;
		bool                   _is_signalled;
		bool                   _isAutoReset;
	};

#endif


class ScopedLocker
{
    public :
    explicit ScopedLocker(Locker &l): _binded(l)
    {
        _binded.lock();
    }

    void forceUnlock()
    {
        _binded.unlock();
    }

    ~ScopedLocker()
    {
        _binded.unlock();
    }
    Locker &_binded;
};

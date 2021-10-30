#ifndef __MY_SIGNAL_H__
#define __MY_SIGNAL_H__

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <signal.h>
#include <cerrno>
#include <stdexcept>
#include <csignal>
#if defined(_MSC_VER)
#include <io.h>
#endif

#if !defined(_MSC_VER)
#include <unistd.h>
#endif

using namespace std;

// Determine if sigaction is available
#if __APPLE__ || _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _POSIX_SOURCE
    #define HAS_SIGACTION
#endif

static volatile sig_atomic_t g_signal_status = 0;

//public handler
#ifdef HAS_SIGACTION
       static struct sigaction old_action;
//set 
       inline struct sigaction set_sigaction(int signal_value,const struct sigaction &action)
       {
           struct sigaction old_action;
           ssize_t ret = sigaction(signal_value,&action,&old_action);

           if(-1 == ret)
           {
               const size_t error_length = 1024;
               char error_string[error_length];

                #ifndef _WIN32
                    #if (defined(_GNU_SOURCE) && !defined(ANDROID) &&(_POSIX_C_SOURCE >= 200112L))
                         char *msg = strerror_r(errno, error_string, error_length);
                         if (msg != error_string) 
                         {
                              strncpy(error_string, msg, error_length);
                              msg[error_length - 1] = '\0';
                         }
                     #else
                         int error_status = strerror_r(errno, error_string, error_length);

                          if (error_status != 0) 
                          {
                              throw std::runtime_error("Failed to get error string for errno: " +
                                                       std::to_string(errno));
                          }

                     #endif
               #else 
                    strerror_s(error_string, error_length, errno);
               #endif
                // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
                throw std::runtime_error(
                  std::string("Failed to set SIGINT signal handler: (" + std::to_string(errno) + ")") +
                  error_string);
                // *INDENT-ON*
           }
           return old_action;
       }
    
    
//int         
       inline void trigger_interrupt_guard_condition(int signal_value) 
       {
           g_signal_status = signal_value;
           signal(signal_value, SIG_DFL);
       }
    
//handle 
       inline void signal_handler(int signal_value, siginfo_t *siginfo, void *context)
       {
           if (old_action.sa_flags & SA_SIGINFO) 
           {
                if (old_action.sa_sigaction != NULL) {
                  old_action.sa_sigaction(signal_value, siginfo, context);
                }
           } 
           else 
           {
                if ( old_action.sa_handler != NULL &&  // Is set
                     old_action.sa_handler != SIG_DFL &&  // Is not default
                      old_action.sa_handler != SIG_IGN )     // Is not ignored
                { 
                  old_action.sa_handler(signal_value);
                }       
           }
           trigger_interrupt_guard_condition(signal_value);
       }
#else 
       typedef void (* signal_handler_t)(int);
       static signal_handler_t old_signal_handler = 0;

//set 
       inline signal_handler_t set_signal_handler(int signal_value, signal_handler_t signal_handler)
       {
           signal_handler_t old_signal_handler = std::signal(signal_value, signal_handler);

            // NOLINTNEXTLINE(readability/braces)
            if (old_signal_handler == SIG_ERR)
            {
                const size_t error_length = 1024;
                // NOLINTNEXTLINE(runtime/arrays)
                char error_string[error_length];
                #ifndef _WIN32
                    #if (defined(_GNU_SOURCE) && !defined(ANDROID) &&(_POSIX_C_SOURCE >= 200112L))
                        char *msg = strerror_r(errno, error_string, error_length);

                        if (msg != error_string) 
                        {
                            strncpy(error_string, msg, error_length);
                            msg[error_length - 1] = '\0';
                        }

                     #else
                        int error_status = strerror_r(errno, error_string, error_length);

                        if (error_status != 0) 
                        {
                            throw std::runtime_error("Failed to get error string for errno: " +
                                           std::to_string(errno));
                        }

                    #endif
                #else 
                    strerror_s(error_string, error_length, errno);
                #endif
                 // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
                throw std::runtime_error(
                  std::string("Failed to set SIGINT signal handler: (" + std::to_string(errno) + ")") +
                  error_string);
                // *INDENT-ON*
            }
           return old_signal_handler;        
       }
//int         
       inline void trigger_interrupt_guard_condition(int signal_value) 
       {
           g_signal_status = signal_value;
           signal(signal_value, SIG_DFL);
       }
//handle 
       inline void signal_handler(int signal_value)
       {
             // TODO(wjwwood): remove? move to console logging at some point?
            printf("signal_handler(%d)\n", signal_value);
            if (old_signal_handler) 
            {
                old_signal_handler(signal_value);
            }
            trigger_interrupt_guard_condition(signal_value);
       }
#endif

//内容
namespace nvilidar
{
//signal init
    inline void SigInit(void) 
    {
        #ifdef HAS_SIGACTION
          struct sigaction action;
          memset(&action, 0, sizeof(action));
          sigemptyset(&action.sa_mask);
          action.sa_sigaction = ::signal_handler;
          action.sa_flags = SA_SIGINFO;
          ::old_action = set_sigaction(SIGINT, action);
          set_sigaction(SIGTERM, action);

        #else
          ::old_signal_handler = set_signal_handler(SIGINT, ::signal_handler);
          // Register an on_shutdown hook to restore the old signal handler.
        #endif
    }
//check sig is ok
    inline bool isOK() 
    {
        return g_signal_status == 0;
    }
//shutdown process
    inline void shutdownNow() 
    {
        trigger_interrupt_guard_condition(SIGINT);
    }
//file exsists 
    inline bool fileExists(const std::string filename)
    {
        #ifdef _WIN32
          struct _stat info = {0};
          int ret = _stat(filename.c_str(), &info);
        #else
          struct stat info = {0};
          int ret = stat(filename.c_str(), &info);
        #endif
          return (ret == 0);
          /*return 0 == _access(filename.c_str(), 0x00 ); // 0x00 = Check for existence only!*/
    }
}

#endif

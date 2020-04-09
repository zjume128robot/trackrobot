#ifndef __MY_LOG_H__
#define __MY_LOG_H__

#include<stdio.h>
#include<string>
#include <stdarg.h>
#include<string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <map>
#include <queue>
//#include <mutex>
#include"boost/thread.hpp"
#include <iostream>
#include "logbasedef.hpp"
const char kHeaderTimestamp[] = "%04u-%02u-%02u %02u:%02u:%02u.%03u ";
const char kHeaderThreadId[] = "[%04x] ";
const char kHeaderFileAndLine[] = "[%s:%d] ";
// 函数名作为日志头的最后一个节， 打上冒号用于区别正文
const char kHeaderFunction[] = "[%s]: "; 
const char kHeaderLogLevel[] = "[%s] ";
const char kLogFilenameSuffix[] = "%04u%02u%02u-%02u%02u%02u.log";
#define kMaxBufferSize				( 3* 1024)
#define kLogFilenameSuffixSize		(32)
#ifdef _WIN32
#define kSystemSlash  '\\'
#else
#define kSystemSlash  '/'
#endif

namespace DG_LOG
{
  using std::string;
  enum Log_LEVEL{Log_Null,Log_Error,\		  
	Log_Warning,Log_Debug,Log_Info,Log_Everythig};
  class Log
  {
  public:
      Log();
      Log( std::string module );
    ~Log();
    void print(unsigned int level,const char* filename,unsigned int\
		 line,const char*  function,const char*  format,va_list ap);
   	void print(unsigned int level, const char* filename, unsigned int\
		 line,const char* function, const char *text);
    bool setLogFile(const char* name);
    const char* getLogFile();
	inline  void setLogLevel(Log_LEVEL lev){
      log_level = lev;
    }
    inline Log_LEVEL getLogLevel(){
      return log_level;
    }
    void clear_log_file(unsigned int reserve_day);
    const char* level_string(unsigned int level);
    int sprintf_filename_suffix( char* buffer,int buffer_size,const\
		 SystemTime& system_time);
    int sprintf_timestamp(unsigned int head_type,char* buffer,int\
		 buffer_size,const SystemTime& system_time);
    int sprintf_thread_id(unsigned int head_type,char* buffer,int\
		 buffer_size, unsigned int thread_id);
	const char* trim_filename(const char* filename);
    int sprintf_file_and_line(
        unsigned int head_type,
        char* buffer,
        int buffer_size,
        const char* filename,
        unsigned int line);
    int sprintf_level(
        unsigned int /*head_type*/,
        char* buffer,
        int buffer_size,
        unsigned int level);
    void hexdump(
        unsigned int level,
        const char*  filename,
        unsigned int line,
        const char*  function,
        const void*  data,
        int size);
	int logError(const char* fmt, ...);
	int logWarning( bool putScreen, const char* fmt, ...);
    void set_directory(const char* path);
    void set_subdir(const char *subdir);
    void set_prefix_name(const char* prefix_name);
    void set_level(unsigned int level);
    void set_storage_type(unsigned int storage_type);
    void set_field_type(unsigned int field_type);
    void set_reserve_days(int days);
    void set_escape_change_logfile(unsigned long milliseconds);
    std::basic_string<char> module() const;
    void module(std::basic_string<char> &module_name) const;
    std::string get_log_directory() const;
    std::string get_sub_directory() const;
    
    int logDirect(Log_LEVEL lev, bool putScreen, const char* fmt, ...);
    int logDebug( bool putScreen, const char* fmt, ...);
    int logInfo( bool putScreen, const char* fmt, ...);
    int logEverythin( bool putScreen, const char* fmt, ...);
  private:
    static Log *_instance;
    bool enableLog;
    bool isOpened;
		SystemTime last_time_;
    const char* fileName;
    Log_LEVEL log_level;
    boost::mutex lock_;
    //std::mutex lock_;
    unsigned int storage_type_ ;
    unsigned int field_type_ ;
    unsigned int level_ ;
    unsigned int reserve_days_ ;  // 日志保留天数，默认30天
    int filename_depth_ ;
    std::string log_directory_;
    std::string log_sub_dir_;		// 子目录
    std::string prefix_name_;
    char old_suffix_[kLogFilenameSuffixSize];
    char new_suffix_[kLogFilenameSuffixSize];
    FILE* file_ ;
    int fd_ ;
    FILE* file;
    unsigned long using_file_timestap_ ;
    unsigned long esacpe_milliseconds_ ;
	// 缺省1天变更文件
    unsigned long milliseconds_change_logfile_;
    std::string module_;
	private:
	int sprintf_function(
        unsigned int head_type,
        char* buffer,
        int buffer_size,
        const char* function);
    std::string get_log_filename(time_t seconds);
    std::string get_log_filename(const char* suffix);
    void write(unsigned int level,const char* message,int size,
        const SystemTime& system_time);
    void write_to_screen(unsigned int level, const char *message,\
		int size);
    void write_to_stderror(unsigned int level, const char *message,\
		int size);
    void wirte_to_file(const char*, const SystemTime&);
    int format_line_head(
        char* buffer,
        int size,
        unsigned int level,
        const char*  filename,
        unsigned int line,
        const char*  function,
        const SystemTime& system_time);
    //Log();
    //Log(const Log&)   { }
    //Log& operator=(const Log &)   {}
  };
 // Log * Log::_instance=NULL;
}


#endif

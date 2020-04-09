#pragma once

#include <string>
#include <iomanip>
void set_logfile_change( const char *module, unsigned long milliseconds );
void output_log_impl( const char *module, unsigned int level, const char*  filename, unsigned int line, \
                      const char*  function, const char*  format, ... );
void set_subdir( const char *module, const char *subdir );
namespace log_base{
class hex {
public:
	hex(unsigned char c) : __cb(1) { __byte8 = c; }
	hex(char c) : __cb(1) { __character = c; }
	hex(unsigned int n) : __cb(4) { __integer = n; }
	hex(int n) : __cb(4) { __integer = n; }
	hex(unsigned short n) : __cb(2) { __byte16 = n; }
	hex(short n) : __cb(2) { __word = n; }
	hex(unsigned long n) : __cb(4) { __byte32 = n; }
	hex(long n) : __cb(4) { __dword = n; }
	hex(void *ptr) : __cb(4) { __ptr = ptr; }
	union {
		unsigned char __byte8;
		char __character;
		unsigned int __byte32;
		int __integer;
		unsigned short __byte16;
		short __word;
		long __dword;
		void *__ptr;
		int __auto_t;
	};
	int __cb;

#if _MSC_VER >= 1800
	hex(const hex &) = delete;
	hex(const hex &&) = delete;
	hex &operator=(const hex &) = delete;
#else
private:
	hex(const hex &);
	hex &operator=(const hex &);
#endif
};
}
class logger {
public:
	logger(const char *module, unsigned long milliseconds);
	logger(const char *module, unsigned int level, const char *file, unsigned int line, const char *function);
	~logger();
#if _MSC_VER >= 1800
	//logger( const logger & ) = delete;
	//logger( const logger && ) = delete;
	//logger &operator=( const logger & ) = delete;
#else
private:
	logger(const logger &);
	logger &operator=(const logger &);
#endif
public:
	logger &operator()(const char *name);

	logger &operator<<(const char *str);
	logger &operator<<(const wchar_t *str);

	logger &operator<<(int n);
	logger &operator<<(unsigned int n);

	logger &operator<<(short n);
	logger &operator<<(unsigned short n);

	logger &operator<<(long n);
	logger &operator<<(unsigned long n);

	logger &operator<<(long long n);
	logger &operator<<(unsigned long long n);

	logger &operator<<(const std::basic_string<char> &str);

	logger &operator<<(float f);
	logger &operator<<(double lf);

	logger &operator<<(void *ptr);
	logger &operator<<(void **ptr);

    logger &operator<<(const log_base::hex &ob);


    //static void set_subdir(const char *module, const char *subdir);
    //static void set_mode(const int mode);

	// 日志允许继续派生
protected:
	char __str[3072]; // 2KB发生过越界
	char __file[255];
	char __function[128];
	unsigned int __line;
	char __module[128];
	unsigned int __level;
	std::streamsize __strsize;
};

enum logger_level_impl  // 日志输出级别
{
	__kStartup = 1,			// 启动级别
	__kShutdown = 2,		// 关闭级别
	__kInfo = 4,			// 普通信息级别
	__kWarning = 8,			// 警告级别
	__kTrace = 0x10,		// 跟踪级别
	__kError = 0x20,		// 错误级别
	__kDebug = 0x40,		// 调试级别
	__kDebugCaller = 0x80,
};

#define logfile_change_milliseconds(module, milliseconds) logger(module, milliseconds)
#define output_debug_information(module, level) (module ? logger(module, level, __FILE__, __LINE__, __FUNCTION__) : logger("ProcessTrunk", level, __FILE__, __LINE__, __FUNCTION__))
#define log_fxchange(module, milliseconds) logfile_change_milliseconds(module,milliseconds)
#define log_output(module, level) output_debug_information(module, level)
#define log_subdir(module, subdir)logger::set_subdir(module, subdir)
#define log_setmod(mode)logger::set_mode(mode)


#define dbgerror(module)	output_debug_information(module,__kError)
#define dbgwarning(module)	output_debug_information(module,__kWarning)
#define dbginfo(module)		output_debug_information(module,__kInfo)

const unsigned long __kLogSwitchMinite = 60 * 1000;
const unsigned long __kLogSwitchHour = 60 * __kLogSwitchMinite;
const unsigned long __kLogSwitchDay = 24 * __kLogSwitchHour;
const unsigned long __kLogSwitchMonth = 30 * __kLogSwitchDay;

#define LOGS_SET_SWITCH_TIME_MS(module,ms) logfile_change_milliseconds(module, ms)

#define LOGS_INFO(module) output_debug_information(module, __kInfo)
#define LOGS_WARN(module) output_debug_information(module, __kWarning)
#define LOGS_DEBUG(module) output_debug_information(module,__kDebug)
#define LOGS_ERROR(module) output_debug_information(module,__kError)

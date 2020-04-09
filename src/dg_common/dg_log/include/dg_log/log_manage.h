#pragma once

#include <memory>
#include <map>
#include <queue>
//#include <thread>
#include <semaphore.h>
#include "dg_log/Log.h"
#include "dg_log/singleton_instance.hpp"

#define ASYNCH_MODE				(1)
#define SYNCHRONOUS_MODE		(0)
using namespace DG_LOG;
struct log_context
{
	unsigned int level_;
	std::string  filename_;
	unsigned int line_;
	std::string function_;
	std::string text_;
    //std::shared_ptr<Log> slogptr_;
    boost::shared_ptr<Log> slogptr_;
};

class log_object_manager {
	log_object_manager();
	~log_object_manager();
	void async_proc();
    void post_asynch_task( const boost::shared_ptr<Log> &slogptr, unsigned int level, const char*  filename,
		unsigned int line, const char*  function, const char*  format, va_list ap );

    void set_log_directory( boost::shared_ptr<Log> &slogptr);
    //friend class singleton_instance<log_object_manager>;
    void parse_log_directory(boost::shared_ptr<Log> &slogptr, const std::string &ext_dir);
	int parse_module_name(const char *module_ptr, std::string &ext_dir, std::string &name);
    boost::shared_ptr<Log> reference_log_object( const char *module );

public:
    static log_object_manager* instance() ;
	void write_log( const char *module, unsigned int level, const char*  filename, 
		unsigned int line, const char*  function, const char*  format, va_list ap );
	void set_logfile_change( const char *module, unsigned long milliseconds );
	void set_subdir( const char *module, const char *subdir );
	void set_mode( int mode );

private:
   boost::mutex __lock_log_objects;
   // static std::recursive_mutex __lock_log_;
    //static boost::recursive_mutex __lock_log_;
    std::map<std::string, boost::shared_ptr<Log> >		__log_object_map;
     static log_object_manager* instance_;
    std::queue<boost::shared_ptr<log_context> >	__task_que;
 //   std::thread::native_waitable_handle<0>		__notify_evt;
    bool bSig_;
    sem_t sig_t;
    //std::thread		*__async = nullptr;
    boost::thread *__async ;

    int mode_ ;
};
//

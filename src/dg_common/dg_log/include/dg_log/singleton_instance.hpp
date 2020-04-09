#pragma once
//#include <mutex>
#include <boost/thread.hpp>
//template <class T, class LOCK = std::recursive_mutex>
template <class T, class LOCK = boost::recursive_mutex>
class singleton_instance :
	public T {
public:
	//在第一次实例化的时候使用DoubleCheckNull方式检测并发性
	static T* instance() {
		if (!__instance) {
            boost::lock_guard<boost::recursive_mutex> guard( __locker );
			if (!__instance) {
				__instance = new T;
			}
		}
		return __instance;
	}

	static void associate(T *ptr)
	{
		if (!__instance) {
            //std::lock_guard<std::recursive_mutex> guard( __locker );
           boost::lock_guard<boost::recursive_mutex> guard( __locker );
			if (!__instance) {
				__instance = ptr;
				__associated = 1;
			}
		}
	}

	static void release() {
		if (__instance && !__associated) {
            //std::lock_guard<std::recursive_mutex> guard( __locker );
             boost::lock_guard<boost::recursive_mutex> guard( __locker );
			if (__instance && !__associated) {
				delete __instance;
				__instance = NULL;
			}
		}
	}

private:
	singleton_instance();
	~singleton_instance();

	singleton_instance( const singleton_instance& singletonhandler );
	singleton_instance& operator=( const singleton_instance& singletonhandler );

	static T* __instance;
	static LOCK __locker;
	static int __associated;
};

template <class T, class LOCK>
T *singleton_instance<T, LOCK>::__instance = NULL;

template <class T, class LOCK>
LOCK singleton_instance<T, LOCK>::__locker;

template <class T, class LOCK>
int singleton_instance<T, LOCK>::__associated = 0;

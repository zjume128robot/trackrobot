#pragma once
#include <memory>
namespace std {
#if 0
template <class T>
class unique_ptr {
public:
	unique_ptr() : __ptr( NULL ) {}
	unique_ptr( const T *ptr ) : __ptr( ptr ) {}
	~unique_ptr() { release(); }

	unique_ptr<T> &operator =( const unique_ptr<T> &ptr )	
	{
		reset(ptr.get());
		return *this;
	}

	void reset( T *ptr ) 
	{ 
		T *old = __ptr;
		__ptr = ptr; 
		if (old != __ptr && old) {
			delete old;
		}
	}

	void swap(unique_ptr<T> &other)
	{
		T *mine = __ptr;
		__ptr = other.get();
		other.__ptr = mine;
	}

	template<class U>
	void swap(unique_ptr<U> &other)
	{
		T *mine = __ptr;
		__ptr = reinterpret_cast<T*>(other.get());
		other.__ptr = reinterpret_cast<U*>(mine);
	}

	T *operator->() { return __ptr; }
	T *operator->() const { return __ptr; }

	T &operator*() { return *__ptr; }
	T &operator*() const { return *__ptr; }

	operator bool() const { return ( NULL != __ptr ); }

	T *get() const { return __ptr; }

	void release()
	{
		if (__ptr) {
			delete __ptr;
		}
		__ptr = NULL;
	}

	unique_ptr<T> &operator =( T *ptr ) 
	{
		reset(ptr);
		return *this;
	}
private:
	T *__ptr;

private:
	unique_ptr(const unique_ptr<T> &rf) {}

};
#endif
} // namespace std


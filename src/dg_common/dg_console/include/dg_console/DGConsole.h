#ifndef DGConsole_H
#define DGConsole_H
#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <histedit.h>
#include <ros/ros.h>

typedef boost::function<void (const char*)> onCmdType;

class DGConsole
{
public:
  DGConsole(const char* consoleName, onCmdType fun);
  ~DGConsole();

protected:
  void stopConsole();

private:
  EditLine *el;
  History *myhist;
  //char *prompt(EditLine *e);

  boost::thread *console_thread;
  volatile bool run_console_thread;
  void console_thread_fun(onCmdType fun);

};



#endif // DGConsole_H

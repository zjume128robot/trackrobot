#include <dg_console/DGConsole.h>
#include <pthread.h>
#include <histedit.h>
#include <string>
using namespace std;
namespace DGConsoleName
{
  string name = "test";
}


char * prompt(EditLine *e) {
  return (char* )(DGConsoleName::name.c_str());
}

DGConsole::DGConsole(const char *consoleName, onCmdType fun)
{
  DGConsoleName::name = string(consoleName) + string("> ");
  run_console_thread = true;
  console_thread = new boost::thread(boost::bind(&DGConsole::console_thread_fun, this, fun));
}

DGConsole::~DGConsole()
{
  stopConsole();
  if (console_thread)
  {
    delete console_thread;
    console_thread = NULL;
  }

  history_end(myhist);
  el_end(el);
}

void DGConsole::stopConsole()
{
    if (run_console_thread)
    {
        run_console_thread = false;
        pthread_cancel(console_thread->native_handle());
    }
}

void DGConsole::console_thread_fun(onCmdType fun)
{
  const char *line;
  char buf[128];
  HistEvent ev;

  //ROS_INFO("begin DGConsole::console_thread_fun");
  
  el = el_init(DGConsoleName::name.c_str(), stdin, stdout, stderr);
  el_set(el, EL_PROMPT, &prompt);//boost::bind(&DGConsole::prompt, this, _1));
  el_set(el, EL_EDITOR, "emacs");

  myhist = history_init();
  if(myhist == 0)
  {
    ROS_ERROR("console_thread_fun: history_init() for cmd line edit error\n");
  }
  history(myhist, &ev, H_SETSIZE, 800);
  el_set(el, EL_HIST, history, myhist);

  int count;
  while (run_console_thread)
  {
    line = el_gets(el, &count);
    if (count > 1)
    {
      strncpy(buf, line, count);
      buf[count - 1] = '\0';
      history(myhist, &ev, H_ENTER, line);
      fun((const char*) buf);
    }
  }
}

/*
char* DGConsole::prompt(EditLine *e)
{
  return (char* )(DGConsoleName::name.c_str());
}
*/

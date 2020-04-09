#ifndef EXAMPLE_H
#define EXAMPLE_H
#include<dg_console/DGConsole.h>

class TestDGConsole :public DGConsole
{
public:

  TestDGConsole();
  TestDGConsole(const char * name);
  ~TestDGConsole();

  const char * _name;
  void oncmd(const char* cmd);
};


#endif // EXAMPLE_H

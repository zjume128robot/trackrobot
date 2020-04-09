#include"example.h"
#include<iostream>
#include<boost/thread.hpp>
#include<ros/ros.h>

#include <unistd.h>


using namespace std;
TestDGConsole::TestDGConsole():DGConsole("Test", boost::bind(&TestDGConsole::oncmd, this, _1))
{
  _name = "Test";
  //Do whatever you want
}

TestDGConsole::TestDGConsole(const char * name):DGConsole(name, boost::bind(&TestDGConsole::oncmd, this, _1))
{
  _name = name;
  //Do whatever you want
}




TestDGConsole::~TestDGConsole()
{
  //Do what ever you want
}

void TestDGConsole::oncmd(const char *cmd)
{

  cout<<_name<<">--"<<cmd<<endl;
  // Your cmd processing funtion
}


int main(int argc, char** argv)
{
  ros::init(argc,argv, "testConsole");
  if(argc == 2)
  {
    TestDGConsole a(argv[1]);
  }
  else
    TestDGConsole a("test");
}

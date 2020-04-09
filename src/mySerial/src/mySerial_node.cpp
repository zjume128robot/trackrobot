//serial_port.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;
    
    // create a serial object
    serial::Serial sp;
    // create timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    // configure serial port name
    sp.setPort("/dev/ttyS4");
    // configure paud rate
    sp.setBaudrate(115200);
    // setup timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    // judge whether the serial port is opened successfully
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("dev/ttyS0 is opened.");
    }
    else
    {
        return -1;
    }
    
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        uint8_t buf[1024];
        buf[0] = 'h';
        buf[1] = 'e';
        buf[2] = 'l';
        buf[3] = 'l';
        buf[4] = '0';
        sp.write(buf, 5);
        loop_rate.sleep();
    }

    /*
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);
            
            for(int i=0; i<n; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
            //把数据发送回去
            sp.write(buffer, n);
        }
        loop_rate.sleep();
    }
    */
    
    //关闭串口
    sp.close();
 
    return 0;
}


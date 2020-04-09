#ifndef __avoidance_H__PPPPP____
#define __avoidance_H__PPPPP____

#include <ros/ros.h>
#include <dg_console/DGConsole.h>
#include <std_msgs/String.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>


class avoidance: public DGConsole
{

public:
    avoidance();
    ~avoidance();

private:

    enum OBState {Clear, Near, ObstacleLeft, ObstacleRigt,Obstacle, Stop};

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber laserSub;
    void onLaser(const sensor_msgs::LaserScanConstPtr &laser);

    ros::Subscriber cmdSub;
    void onCmd(const std_msgs::StringConstPtr &cmd);

    ros::Publisher baseCmdPub;

    boost::thread* controlThread;
    bool running;
    void avoid();

    void processCmd(const char * cmd);

    OBState ObstacleState;
    ros::Time startTime;

    double stopDist;
    double dangerDist;
    double safeDist;

    double fastSpeed;
    double slowSpeed;
    double moveTime;

    double avoidSpeed;
    double halfWidth;
    int minNum;


    double x[1141];
    double y[1141];


};


#endif //__avoidance_H__PPPPP____

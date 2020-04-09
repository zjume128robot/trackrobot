#include<avoidance/avoidance.h>
#include<iostream>
#include <dg_include/xform.h>
using namespace std;

avoidance::avoidance():
DGConsole("avd", boost::bind(&avoidance::processCmd, this, _1)),
    private_nh("~")
{
    private_nh.param("stopDist", stopDist, 0.3);
    private_nh.param("dangerDist", dangerDist, 0.5);
    private_nh.param("safeDist", safeDist, 1.2);
    private_nh.param("fastSpeed", fastSpeed, 0.5);
    private_nh.param("slowSpeed", slowSpeed, 0.3);
    private_nh.param("moveTime", moveTime, 60.0);
    private_nh.param("avoidSpeed", avoidSpeed, 0.3);
    private_nh.param("halfWidth", halfWidth, 0.4);
    private_nh.param("minNum", minNum, 0);

    laserSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &avoidance::onLaser, this);
    cmdSub = nh.subscribe<std_msgs::String>("avoidance_cmd", 10, &avoidance::onCmd, this);
    baseCmdPub = nh.advertise<std_msgs::String>("base_cmd", 1);

    ObstacleState = Stop;
    running = false;

    controlThread = NULL;
}

avoidance::~avoidance()
{
    if(controlThread)
    {
        running = false;
        usleep(100000);
        delete controlThread ;
        controlThread = NULL;
    }

}


void avoidance::onCmd(const std_msgs::StringConstPtr &cmd)
{
    processCmd(cmd->data.c_str());
}


void avoidance::processCmd(const char *cmd)
{
    ROS_INFO("avoidance processCmd recv cmd: %s\n", cmd);
    if(PEEK_CMD(cmd, "exit"))
        ros::requestShutdown();

    else if(PEEK_CMD(cmd, "s"))
    {
        if(!running)
        {
            running = true;
            if(controlThread)
            {
                running = false;
                usleep(100000);
                delete controlThread ;
                controlThread = NULL;
            }

            controlThread = new boost::thread(boost::bind(&avoidance::avoid, this));
        }

    }
    else if(PEEK_CMD(cmd, "t"))
    {
        running = false;
        if(controlThread)
        {
            running = false;
            usleep(100000);
            delete controlThread ;
            controlThread = NULL;
        }
    }
    else
    {
        ROS_ERROR("Unknown Cmd [%s]\n", cmd);
    }

}

void avoidance::onLaser(const sensor_msgs::LaserScanConstPtr &laser)
{
    int num = (laser->angle_max - laser->angle_min) / laser->angle_increment;
    double angle = laser->angle_min;

    for(int i = 0; i < num; i++)
    {
        if(laser->ranges[i] < 0.01)
        {
            x[i] = safeDist + 1.5;
            y[i] = safeDist + 1.5;
        }
        else
        {
            x[i] = cos(angle) * laser->ranges[i];
            y[i] = sin(angle) * laser->ranges[i];
        }

        angle += laser->angle_increment;
    }

    int leftNum = 0;
    int rightNum = 0;

    OBState state = Clear;

    for(int i = 0; i < num; i++)
    {
        if(y[i] > -halfWidth && y[i] < halfWidth)
        {
            if(x[i] <= stopDist)
            {
                state = Stop;
                break;
            }
            else if(x[i] <= dangerDist)
            {
                if(i < 1141 / 2)
                    leftNum++;
                else
                    rightNum++;
                if(state < Obstacle)
                    state = Obstacle;
            }
            else if(x[i] <= safeDist)
            {
                if(state < Near)
                    state = Near;
            }
            //   ROS_INFO("(%d)[%.2f %.2f]",i, x[i], y[i]);
        }


    }

    if(state == Obstacle)
    {
        if(leftNum > minNum || rightNum > minNum)
        {
            if(leftNum > rightNum)
                state = ObstacleLeft;
            else
                state = ObstacleRigt;
        }
        else
            state = Near;
    }

    ObstacleState = state;
    switch(ObstacleState)
    {
    case Clear:ROS_INFO("**Clear**");break;
    case Near:ROS_INFO("Near");break;
    case ObstacleLeft:ROS_WARN("ObstacleLeft");break;
    case ObstacleRigt:ROS_WARN("**ObstacleRigt**");break;
    case Stop:ROS_ERROR("Stop");break;
    default:
        ROS_INFO("Unknown");
    }
}

void avoidance::avoid()
{
    double totalTime = 0.0;
    ros::Rate loopRate(20);
    float Xspeed = 0.0;
    float Yspeed = 0.0;
    bool going = false;
    while(running && ros::ok() && totalTime < moveTime)
    {
        switch(ObstacleState)
        {
        case Clear:
            Xspeed = fastSpeed;
            Yspeed = 0.0;
            if(!going)
            {
                going = true;
                startTime = ros::Time::now();
            }
            break;
        case Near:
            Xspeed = slowSpeed;
            Yspeed = 0.0;
            if(!going)
            {
                going = true;
                startTime = ros::Time::now();
            }
            break;
        case ObstacleLeft:
            Xspeed = 0.0;
            Yspeed = avoidSpeed;
            if(going)
            {
                going = false;
                totalTime += (ros::Time::now() - startTime).toSec();
            }
            break;
        case ObstacleRigt:
            Xspeed = 0.0;
            Yspeed = -avoidSpeed;
            if(going)
            {
                going = false;
                totalTime += (ros::Time::now() - startTime).toSec();
            }
            break;
        case Stop:
            Xspeed = 0.0;
            Yspeed = 0.0;
            if(going)
            {
                going = false;
                totalTime += (ros::Time::now() - startTime).toSec();
            }
            break;
        default:
            ROS_INFO("Unknown");
        }

        char cmd[20];
        sprintf(cmd, "m %.2f %.2f 0.0", Xspeed, Yspeed);
        std_msgs::String str;
        str.data = std::string(cmd);
        baseCmdPub.publish(str);
        loopRate.sleep();
    }

    running = false;


    std_msgs::String str;
    str.data = std::string("m 0.0  0.0 0.0");
    baseCmdPub.publish(str);

}


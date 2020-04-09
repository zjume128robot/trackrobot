#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <math.h>
 
class LaserChooser
{
public:
  LaserChooser()
  {
    //Topic you want to publish
    msg_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1000);
 
    //Topic you want to subscribe
    msg_sub_ = nh_.subscribe("velodyne_points", 1000, &LaserChooser::ChooserCb, this);  
  }
 
  void ChooserCb(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    sensor_msgs::LaserScan output;
    double minangle=M_PI;
    double maxangle=-M_PI;  

    output.header = msg->header;
    //output.angle_min = -M_PI;
    //output.angle_max = M_PI;
    double angle_increment_=M_PI / 900;
    output.angle_increment =  M_PI / 2700;
    output.time_increment = 0.0;
    output.scan_time = 1.0/30.0;
    output.range_min = 0.5;
    output.range_max = 100.0;

    output.ranges.resize(5400);
    output.intensities.resize(5400);
    int k=0;
    for (sensor_msgs::PointCloud2ConstIterator<float>
        iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
        iter_x != iter_x.end();
        ++iter_x, ++iter_y, ++iter_z)
    {
		if (msg->data[k+20] == 7){
			double range = hypot(*iter_x, *iter_y);
			if (range >= output.range_min && range <= output.range_max){
				//overwrite range at laserscan ray if new range is smaller
      			double angle = atan2(*iter_y, *iter_x);
      			if (angle < minangle){
      				minangle = angle;
      			}
      			if (angle > maxangle){
      				maxangle = angle;
      			}
      			int index = (angle + M_PI) / angle_increment_;

      			if (range < output.ranges[index] || output.ranges[index]==0 )
      			{
            		output.ranges[index] = range;
            		unsigned char b[4]={msg->data[k+16],msg->data[k+17],msg->data[k+18],msg->data[k+19]};
            		output.intensities[index] = *((float *)b);
      			}
			}
			
		}
        if (msg->data[k+20] == 10){
			double range = hypot(*iter_x, *iter_y);
			if (range >= output.range_min && range <= output.range_max){
				//overwrite range at laserscan ray if new range is smaller
      			double angle = atan2(*iter_y, *iter_x);
      			if (angle < minangle){
      				minangle = angle;
      			}
      			if (angle > maxangle){
      				maxangle = angle;
      			}
      			int index = (angle + M_PI) / angle_increment_;

      			if (range < output.ranges[index] || output.ranges[index]==0 )
      			{
            		output.ranges[index+1] = range;
            		unsigned char b[4]={msg->data[k+16],msg->data[k+17],msg->data[k+18],msg->data[k+19]};
            		output.intensities[index+1] = *((float *)b);
      			}
			}
			
		}
        if (msg->data[k+20] == 8){
			double range = hypot(*iter_x, *iter_y);
			if (range >= output.range_min && range <= output.range_max){
				//overwrite range at laserscan ray if new range is smaller
      			double angle = atan2(*iter_y, *iter_x);
      			if (angle < minangle){
      				minangle = angle;
      			}
      			if (angle > maxangle){
      				maxangle = angle;
      			}
      			int index = (angle + M_PI) / angle_increment_;

      			if (range < output.ranges[index] || output.ranges[index]==0 )
      			{
            		output.ranges[index+2] = range;
            		unsigned char b[4]={msg->data[k+16],msg->data[k+17],msg->data[k+18],msg->data[k+19]};
            		output.intensities[index+2] = *((float *)b);
      			}
			}
			
		}
        /*if (msg->data[k+20] == 15){
			double range = hypot(*iter_x, *iter_y);
			if (range >= output.range_min && range <= output.range_max){
				//overwrite range at laserscan ray if new range is smaller
      			double angle = atan2(*iter_y, *iter_x);
      			if (angle < minangle){
      				minangle = angle;
      			}
      			if (angle > maxangle){
      				maxangle = angle;
      			}
      			int index = (angle + M_PI) / angle_increment_;

      			if (range < output.ranges[index] || output.ranges[index]==0 )
      			{
            		output.ranges[index+3] = range;
            		unsigned char b[4]={msg->data[k+16],msg->data[k+17],msg->data[k+18],msg->data[k+19]};
            		output.intensities[index+3] = *((float *)b);
      			}
			}
			
		}*/
		k=k+32;
    }
    output.angle_min = minangle;
    output.angle_max = maxangle;
    msg_pub_.publish(output);
  }
 
private:
  ros::NodeHandle nh_; 
  ros::Publisher msg_pub_;
  ros::Subscriber msg_sub_;
 
};
 
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "LaserChooser");
 
  //Create an object of class that will take care of everything
  LaserChooser LCObject;
 
  ros::spin();
 
  return 0;
}

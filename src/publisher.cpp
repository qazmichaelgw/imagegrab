#include "ros/ros.h"
#include "std_msgs/String.h"
#include "detection_msg/object_2D.h"
#include "detection_msg/objects_2D.h"
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ImageMsg");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<detection_msg::objects_2D>("ImageMsg", 1000);
	ros::Rate loop_rate(10);
	int count = 0;
	while (ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss << "Test" << count;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());
	
		detection_msg::objects_2D objs;
		for (int i = 0; i < 3; i ++)
		{
			detection_msg::object_2D obj;
			obj.bbox[0] = 1;
			obj.bbox[1] = 2;
			obj.bbox[2] = 66;
			obj.bbox[3] = count;
			obj.name = "test";
			objs.object_2D_array.push_back(obj);
		}
		pub.publish(objs);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}

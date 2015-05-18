#include <image_transport/image_transport.h>
#include "detection_msg/objects_2D.h"
#include "detection_msg/object_2D.h"
#include "detection_msg/APC_object.h"
#include "detection_msg/APC_object_array.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
const int max_u = 640, max_v = 480;

typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

sensor_msgs::ImageConstPtr imgPtr;
ros::Publisher pub;

int ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image)
{
    // If position is invalid
    if ((height_pos >= depth_image->height) || (width_pos >= depth_image->width))
        return -1;
    int index = (height_pos*depth_image->step) + (width_pos*(depth_image->step/depth_image->width));
    // If data is 4 byte floats (rectified depth image)
    if ((depth_image->step/depth_image->width) == 4) {
        U_FloatConvert depth_data;
        int i, endian_check = 1;
        // If big endian
        if ((depth_image->is_bigendian && (*(char*)&endian_check != 1)) ||  // Both big endian
           ((!depth_image->is_bigendian) && (*(char*)&endian_check == 1))) { // Both lil endian
            for (i = 0; i < 4; i++)
                depth_data.byte_data[i] = depth_image->data[index + i];
            // Make sure data is valid (check if NaN)
            if (depth_data.float_data == depth_data.float_data)
                return int(depth_data.float_data*1000);
            return -1;  // If depth data invalid
        }
        // else, one little endian, one big endian
        for (i = 0; i < 4; i++) 
            depth_data.byte_data[i] = depth_image->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depth_data.float_data == depth_data.float_data)
            return int(depth_data.float_data*1000);
        return -1;  // If depth data invalid
    }
    // Otherwise, data is 2 byte integers (raw depth image)
   int temp_val;
   // If big endian
   if (depth_image->is_bigendian)
       temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];
   // If little endian
   else
       temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);
   // Make sure data is valid (check if NaN)
   if (temp_val == temp_val)
       return temp_val;
   return -1;  // If depth data invalid
}

// Image Callback
void imageCallback(const sensor_msgs::ImageConstPtr& image) {
	imgPtr = image;
}

void msgCallback(const detection_msg::objects_2D::ConstPtr& msg){
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	unsigned len = msg->object_2D_array.size();
	detection_msg::APC_object_array apc;
	apc.frame_id = "name";
	for (unsigned i = 0; i < len; i ++)
	{
		int u, v, h, w;
		std::string str = msg->object_2D_array[i].name;
		u = msg->object_2D_array[i].bbox[0];
		v = msg->object_2D_array[i].bbox[1];
		h = msg->object_2D_array[i].bbox[2];
		w = msg->object_2D_array[i].bbox[3];
		ROS_INFO("Obj: %d I heard: [%s: %d %d %d %d]",i, str.c_str(), u, v, h, w);
		if (imgPtr)
		{
			int nearest_depth = INT_MAX, depth;
			int uu_idx, vv_idx;
			for (int uu = u; uu < u+h; uu ++)
				for (int vv = v; vv < v+w; vv ++)
				{
					if (uu >= 0 && uu < max_u && vv >= 0 && vv < max_v)
					{
						depth = ReadDepthData(uu, vv, imgPtr);
						if (depth < nearest_depth)
						{
							nearest_depth = depth;
							uu_idx = uu;
							vv_idx = vv;
						}
					}
				}
			int range = 4, count = 0;
			float sum_depth = 0;
			for (int uu = std::max(u, uu_idx-range); uu < std::min(u+h, uu_idx+range); uu++)
				for (int vv = std::max(v, vv_idx-range); vv < std::min(v+w, vv_idx+range); vv++)
						{
							if (uu >= 0 && uu < max_u && vv >= 0 && vv < max_v)
							{
								sum_depth += ReadDepthData(uu,vv,imgPtr);
								count ++;
							}
						}
			depth = sum_depth/count;
			ROS_INFO("Depth: %d", depth);
			detection_msg::APC_object obj;
			obj.name = str;
			float f_l = 575.8157, cx = 314.5, cy = 235.5;
			obj.x = (uu_idx-cx)*depth/f_l;
			obj.y = (vv_idx-cy)*depth/f_l;
			obj.z = depth;
			obj.separation = h*depth/f_l;
			ROS_INFO("3D: [%f %f %f %f]", obj.x, obj.y, obj.z, obj.separation);
			apc.objects.push_back(obj);
		}
	}
	if (imgPtr && ros::ok())
	{
		pub.publish(apc);
		ROS_INFO("Publish: %d", (int)apc.objects.size());
		ros::spinOnce();
	}
}

//*** Main ***//
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imagegrab");
    ros::NodeHandle n;
    printf("READY to get image\n");
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, imageCallback);
	ros::Subscriber subMsg = n.subscribe("ImageMsg", 1000, msgCallback);
	pub = n.advertise<detection_msg::APC_object_array>("APC",1000);
    ros::spin();
    return 0;
}

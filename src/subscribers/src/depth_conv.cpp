#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

int x_cord, y_cord;

typedef union U_FloatParse {

    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

int ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image)
{

    // If position is invalid
    if ((height_pos >= depth_image->height) || (width_pos >= depth_image->width))
        return -10000;
    int index = (height_pos*depth_image->step) + (width_pos*(depth_image->step/depth_image->width));
    // If data is 4 byte floats (rectified depth image)
    if ((depth_image->step/depth_image->width) == 4) {

        U_FloatConvert depth_data;
        int i, endian_check = 1;
        // If big endian
        if ((depth_image->is_bigendian && (*(char*)&endian_check != 1)) ||  // Both big endian
           ((!depth_image->is_bigendian) && (*(char*)&endian_check == 1))) {
 // Both lil endian
            for (i = 0; i < 4; i++)
                depth_data.byte_data[i] = depth_image->data[index + i];
            // Make sure data is valid (check if NaN)
            if (depth_data.float_data == depth_data.float_data)
                return int(depth_data.float_data*1000);
            return -1000;  // If depth data invalid
        }
        // else, one little endian, one big endian
        for (i = 0; i < 4; i++) 
            depth_data.byte_data[i] = depth_image->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depth_data.float_data == depth_data.float_data)
            return int(depth_data.float_data*1000);
        return -1000;  // If depth data invalid
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
   return -1000;  // If depth data invalid
}

// Image Callback
void imageCallback(const sensor_msgs::ImageConstPtr& image) {

    int depth = ReadDepthData(x_cord, y_cord, image); // Width = 640, Height = 480
    //ROS_INFO("Depth: %.3f", depth/1000.0);
    std::cout << "Depth: " << depth/1000.0 << std::endl;
}

//*** Main ***//
int main(int argc, char **argv)
{

    std::cin >> x_cord >> y_cord;

    ros::init(argc, argv, "depth_conv");
    ros::NodeHandle n;
    printf("READY to get image\n");
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/sync/disparity", 1, imageCallback);
    ros::spin();
    return 0;
}

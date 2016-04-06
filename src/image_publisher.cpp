#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/package.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    camera_info_manager::CameraInfoManager info_manager(nh, "camera");
    
    sensor_msgs::CameraInfo info;
    std::string info_file = "file://" + ros::package::getPath("simple_image_publisher") + "/config/default_camera.yaml";
    std::cout << info_file << std::endl;
    if(info_manager.validateURL(info_file)) {
        info_manager.loadCameraInfo(info_file);
        info = info_manager.getCameraInfo();
        info.header.frame_id = "world";
    } else {
        ROS_ERROR("invalid camera info");
        return 1;
    }
    
    image_transport::CameraPublisher pub = it.advertiseCamera("camera/image", 1);
    cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    ROS_INFO_STREAM("image[height, width] = (" << image.rows << ", " << image.cols << ")");
    cv::waitKey(30);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    ros::Rate rate(5);
    while(nh.ok()) {
        info.header.stamp = ros::Time::now();
        
        pub.publish(*msg, info);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

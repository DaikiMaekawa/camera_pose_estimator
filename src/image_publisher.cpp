#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    camera_info_manager::CameraInfoManager info_manager(nh, "world");
    
    sensor_msgs::CameraInfo info;
    if(info_manager.validateURL("/home/daikimaekawa/camera.yaml")) {
        info_manager.loadCameraInfo("/home/daikimaekawa/camera.yaml");
        info = info_manager.getCameraInfo();
    }
    
    image_transport::CameraPublisher pub = it.advertiseCamera("camera/image", 1);
    cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::waitKey(30);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    ros::Rate rate(5);
    while(nh.ok()) {
        pub.publish(*msg, info);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

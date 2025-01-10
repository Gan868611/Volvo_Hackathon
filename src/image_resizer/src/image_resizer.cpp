#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageResizer {
public:
    ImageResizer()
        : it_(nh_) // Initialize ImageTransport with the NodeHandle
    {
        // Subscribe to the "raw" image topic with image_transport
        image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,
                                   &ImageResizer::imageCallback, this);

        // Advertise the "resized" image topic with image_transport
        image_pub_ = it_.advertise("/camera/image", 1);
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;      // For image_transport
    image_transport::Subscriber image_sub_;   // ImageTransport subscriber
    image_transport::Publisher image_pub_;    // ImageTransport publisher

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // Convert ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            // Resize the image
            cv::Mat resized_image;
            cv::resize(cv_ptr->image, resized_image, cv::Size(320, 240));

            // Convert the resized image back to a ROS message
            cv_bridge::CvImage out_msg;
            out_msg.header = msg->header;  // Same timestamp and frame ID
            out_msg.encoding = "bgr8";     // Same encoding
            out_msg.image = resized_image;

            // Publish the resized image
            image_pub_.publish(out_msg.toImageMsg());
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_resizer");
    ImageResizer resizer;
    ros::spin();
    return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

class RealCameraNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher image_pub_;
    ros::Publisher compressed_pub_;
    ros::Publisher camera_info_pub_;

    cv::VideoCapture cap_;
    std::string camera_device_;
    int width_;
    int height_;
    int fps_;
    std::string frame_id_;
    bool use_compressed_;  // Publish compressed JPEG (much faster)
    int jpeg_quality_;

    sensor_msgs::CameraInfo camera_info_msg_;

public:
    RealCameraNode() : pnh_("~"), use_compressed_(false), jpeg_quality_(80) {
        // Get parameters
        pnh_.param<std::string>("camera_device", camera_device_, "/dev/video11");
        pnh_.param<int>("width", width_, 640);
        pnh_.param<int>("height", height_, 480);
        pnh_.param<int>("fps", fps_, 30);
        pnh_.param<std::string>("frame_id", frame_id_, "camera");
        pnh_.param<bool>("use_compressed", use_compressed_, false);
        pnh_.param<int>("jpeg_quality", jpeg_quality_, 80);

        // Open camera with V4L2 backend
        cap_.open(camera_device_, cv::CAP_V4L2);

        if (!cap_.isOpened()) {
            ROS_ERROR("Cannot open camera %s", camera_device_.c_str());
            ros::shutdown();
            return;
        }

        // Set camera properties - NV12 format for zero-copy
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('N', 'V', '1', '2'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);  // Minimize latency

        // Verify settings
        int actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        int actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        int actual_fps = cap_.get(cv::CAP_PROP_FPS);

        ROS_INFO("Camera opened: %dx%d @ %d fps", actual_width, actual_height, actual_fps);

        // Publishers with queue_size=1 for minimal latency
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/image_raw", 1);
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);

        setupCameraInfo();

        ROS_INFO("Real Camera Node started (optimized for 30fps)");
    }

    ~RealCameraNode() {
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

    void setupCameraInfo() {
        camera_info_msg_.header.frame_id = frame_id_;
        camera_info_msg_.height = height_;
        camera_info_msg_.width = width_;
        camera_info_msg_.distortion_model = "plumb_bob";

        // Default distortion coefficients (calibrate for your camera)
        camera_info_msg_.D = {-0.151, 0.0798, 0.001003, 0.00102, -0.02};

        // Default intrinsic matrix (calibrate for your camera)
        double fx = 322.5, fy = 325.0, cx = 320.0, cy = 240.0;
        camera_info_msg_.K = {
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        };

        camera_info_msg_.R = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        camera_info_msg_.P = {
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        };
    }

    void run() {
        cv::Mat frame;
        int frame_count = 0;
        ros::Time start_time = ros::Time::now();

        // No rate limiter - run as fast as camera provides frames
        while (ros::ok()) {
            if (!cap_.read(frame)) {
                ROS_WARN_THROTTLE(1.0, "Failed to read frame");
                continue;
            }

            if (frame.empty()) {
                continue;
            }

            ros::Time stamp = ros::Time::now();

            // Prepare image message
            sensor_msgs::Image img_msg;
            img_msg.header.stamp = stamp;
            img_msg.header.frame_id = frame_id_;
            img_msg.height = frame.rows;
            img_msg.width = frame.cols;

            // Optimize encoding based on use case
            if (publish_bgr_) {
                // BGR8 - for color processing
                img_msg.encoding = "bgr8";
                img_msg.is_bigendian = false;
                img_msg.step = frame.cols * 3;

                size_t size = img_msg.step * frame.rows;
                img_msg.data.resize(size);
                memcpy(&img_msg.data[0], frame.data, size);
            } else {
                // MONO8 - for VINS/VIO (fastest)
                cv::Mat gray;
                if (frame.channels() == 3) {
                    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                } else {
                    gray = frame;
                }

                img_msg.encoding = "mono8";
                img_msg.is_bigendian = false;
                img_msg.step = gray.cols;

                size_t size = img_msg.step * gray.rows;
                img_msg.data.resize(size);
                memcpy(&img_msg.data[0], gray.data, size);
            }

            // Update camera info timestamp
            camera_info_msg_.header.stamp = stamp;

            // Publish
            image_pub_.publish(img_msg);
            camera_info_pub_.publish(camera_info_msg_);

            frame_count++;

            // Print stats every 100 frames
            if (frame_count % 100 == 0) {
                double elapsed = (ros::Time::now() - start_time).toSec();
                double avg_fps = frame_count / elapsed;
                ROS_INFO("Published %d frames, avg FPS: %.2f", frame_count, avg_fps);
            }

            ros::spinOnce();
        }

        // Final stats
        double elapsed = (ros::Time::now() - start_time).toSec();
        ROS_INFO("Final stats: %d frames in %.2f seconds (%.2f FPS)",
                 frame_count, elapsed, frame_count / elapsed);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "real_camera_node");

    RealCameraNode node;
    node.run();

    return 0;
}

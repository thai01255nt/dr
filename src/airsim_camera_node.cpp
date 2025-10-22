#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>

class AirSimCameraNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher image_pub_;
    ros::Publisher camera_info_pub_;

    msr::airlib::MultirotorRpcLibClient* airsim_client_;

    // Parameters
    std::string vehicle_name_;
    std::string camera_name_;
    std::string frame_id_;
    double frame_rate_;
    int image_width_;
    int image_height_;

    // Camera intrinsics
    double fx_, fy_, cx_, cy_;
    double k1_, k2_, k3_, p1_, p2_;

    sensor_msgs::CameraInfo camera_info_msg_;

public:
    AirSimCameraNode() : pnh_("~") {
        // Get parameters
        pnh_.param<std::string>("vehicle_name", vehicle_name_, "Copter");
        pnh_.param<std::string>("camera_name", camera_name_, "0");
        pnh_.param<std::string>("frame_id", frame_id_, "camera");
        pnh_.param<double>("frame_rate", frame_rate_, 20.0);
        pnh_.param<int>("image_width", image_width_, 640);
        pnh_.param<int>("image_height", image_height_, 480);

        // Camera intrinsics (default values for 640x480)
        pnh_.param<double>("fx", fx_, 322.5);
        pnh_.param<double>("fy", fy_, 325.0);
        pnh_.param<double>("cx", cx_, 325.0);
        pnh_.param<double>("cy", cy_, 238.0);

        // Distortion coefficients
        pnh_.param<double>("k1", k1_, -0.151);
        pnh_.param<double>("k2", k2_, 0.0798);
        pnh_.param<double>("k3", k3_, -0.02);
        pnh_.param<double>("p1", p1_, 0.001003);
        pnh_.param<double>("p2", p2_, 0.00102);

        // Publishers
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/image_raw", 1);
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);

        // Connect to AirSim
        try {
            airsim_client_ = new msr::airlib::MultirotorRpcLibClient();
            airsim_client_->confirmConnection();
            ROS_INFO("Connected to AirSim");
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to connect to AirSim: %s", e.what());
            ros::shutdown();
            return;
        }

        // Setup camera info message
        setupCameraInfo();

        ROS_INFO("AirSim Camera Node started");
        ROS_INFO("  Vehicle: %s", vehicle_name_.c_str());
        ROS_INFO("  Camera: %s", camera_name_.c_str());
        ROS_INFO("  Frame rate: %.1f Hz", frame_rate_);
        ROS_INFO("  Resolution: %dx%d", image_width_, image_height_);
        ROS_INFO("Optimized for VinsMono on OrangePi 5 Max");
    }

    ~AirSimCameraNode() {
        if (airsim_client_) {
            delete airsim_client_;
        }
    }

    void setupCameraInfo() {
        camera_info_msg_.header.frame_id = frame_id_;
        camera_info_msg_.height = image_height_;
        camera_info_msg_.width = image_width_;
        camera_info_msg_.distortion_model = "plumb_bob";

        // Distortion coefficients [k1, k2, p1, p2, k3]
        camera_info_msg_.D = {k1_, k2_, p1_, p2_, k3_};

        // Intrinsic camera matrix K
        camera_info_msg_.K = {
            fx_, 0.0, cx_,
            0.0, fy_, cy_,
            0.0, 0.0, 1.0
        };

        // Rectification matrix (identity for monocular)
        camera_info_msg_.R = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        // Projection matrix
        camera_info_msg_.P = {
            fx_, 0.0, cx_, 0.0,
            0.0, fy_, cy_, 0.0,
            0.0, 0.0, 1.0, 0.0
        };
    }

    void run() {
        ros::Rate rate(frame_rate_);

        int frame_count = 0;
        ros::Time start_time = ros::Time::now();

        while (ros::ok()) {
            try {
                // Get compressed image from AirSim (PNG format)
                std::vector<uint8_t> png_image = airsim_client_->simGetImage(
                    camera_name_,
                    msr::airlib::ImageCaptureBase::ImageType::Scene
                );

                if (png_image.size() == 0) {
                    ROS_WARN_THROTTLE(5.0, "Received empty image from AirSim");
                    rate.sleep();
                    continue;
                }

                // Decode PNG to OpenCV Mat
                cv::Mat img_bgr = cv::imdecode(png_image, cv::IMREAD_COLOR);

                if (img_bgr.empty()) {
                    ROS_WARN_THROTTLE(5.0, "Failed to decode image from AirSim");
                    rate.sleep();
                    continue;
                }

                // Convert BGR to grayscale (VinsMono uses grayscale)
                cv::Mat img_gray;
                cv::cvtColor(img_bgr, img_gray, cv::COLOR_BGR2GRAY);

                // Resize if needed
                if (img_gray.cols != image_width_ || img_gray.rows != image_height_) {
                    cv::resize(img_gray, img_gray, cv::Size(image_width_, image_height_));
                }

                // Convert to ROS message
                std_msgs::Header header;
                header.stamp = ros::Time::now();
                header.frame_id = frame_id_;

                sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(
                    header,
                    "mono8",
                    img_gray
                ).toImageMsg();

                // Update camera info timestamp
                camera_info_msg_.header.stamp = header.stamp;

                // Publish
                image_pub_.publish(image_msg);
                camera_info_pub_.publish(camera_info_msg_);

                frame_count++;

                // Print stats every 100 frames
                if (frame_count % 100 == 0) {
                    double elapsed = (ros::Time::now() - start_time).toSec();
                    double avg_fps = frame_count / elapsed;
                    ROS_INFO("Published %d frames, avg FPS: %.2f", frame_count, avg_fps);
                }

            } catch (const std::exception& e) {
                ROS_ERROR("Error in main loop: %s", e.what());
            }

            ros::spinOnce();
            rate.sleep();
        }

        // Final stats
        double elapsed = (ros::Time::now() - start_time).toSec();
        ROS_INFO("Final stats: %d frames in %.2f seconds (%.2f FPS)",
                 frame_count, elapsed, frame_count / elapsed);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "airsim_camera_node");

    AirSimCameraNode node;
    node.run();

    return 0;
}

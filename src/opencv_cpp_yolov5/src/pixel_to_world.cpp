#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PointStamped.h>

#include <opencv_cpp_yolov5/BoxCenter.h>

// 假设您已经有相机内参
double fx, fy, cx, cy;

// 将像素坐标转换为相机坐标
cv::Point3d pixelToCamera(const cv::Point2d& pixel, double depth) {
    double x = (pixel.x - cx) * depth / fx;
    double y = (pixel.y - cy) * depth / fy;
    double z = depth;
    return cv::Point3d(x, y, z);
}

// 将相机坐标转换为世界坐标
tf::Vector3 cameraToWorld(const cv::Point3d& camera_point, const tf::Transform& camera_to_world_transform) {
    tf::Vector3 camera_vector(camera_point.x, camera_point.y, camera_point.z);
    tf::Vector3 world_vector = camera_to_world_transform * camera_vector;
    return world_vector;
}

class PixelToWorldNode {
public:
    PixelToWorldNode() {
        camera_info_sub_ = nh_.subscribe("/camera/rgb/camera_info", 1, &PixelToWorldNode::cameraInfoCallback, this);
        depth_image_sub_ = nh_.subscribe("/camera/depth/image_raw", 1, &PixelToWorldNode::depthImageCallback, this);
        yolo_bbox_sub_ = nh_.subscribe("/opencv_cpp_yolov5/box_center", 1, &PixelToWorldNode::yoloCallback, this);
        world_coord_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/opencv_cpp_yolov5/world_coordinates", 1);
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        fx = msg->K[0];
        fy = msg->K[4];
        cx = msg->K[2];
        cy = msg->K[5];
    }

    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void yoloCallback(const opencv_cpp_yolov5::BoxCenterConstPtr& msg) {
        if (depth_image_.empty()) {
            ROS_WARN("Depth image is empty, skipping YOLO callback.");
            return;
        }

        tf::StampedTransform camera_to_world_transform;
        try {
            listener_.waitForTransform("/world", "/camera", ros::Time(0), ros::Duration(10.0));
            listener_.lookupTransform("/world", "/camera", ros::Time(0), camera_to_world_transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        int center_x = msg->x;
        int center_y = msg->y;
        float depth = depth_image_.at<float>(center_y, center_x);

        cv::Point3d camera_point = pixelToCamera(cv::Point2d(center_x, center_y), depth);
        tf::Vector3 world_point = cameraToWorld(camera_point, camera_to_world_transform);

        geometry_msgs::PointStamped world_point_msg;
        world_point_msg.header = msg->header;
        world_point_msg.point.x = world_point.x();
        world_point_msg.point.y = world_point.y();
        world_point_msg.point.z = world_point.z();
        world_coord_pub_.publish(world_point_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber depth_image_sub_;
    ros::Subscriber yolo_bbox_sub_;
    ros::Publisher world_coord_pub_;
    tf::TransformListener listener_;
    cv::Mat depth_image_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pixel_to_world_node");
    PixelToWorldNode node;
    ros::spin();
    return 0;
}
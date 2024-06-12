#include <fstream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Header.h>
#include <zal_yolov5/BoundingBox.h>
#include <zal_yolov5/BoundingBoxes.h>

std::vector<std::string> load_class_list(const std::string &class_list_path)
{
    std::vector<std::string> class_list;
    std::ifstream ifs(class_list_path);
    std::string line;
    while (getline(ifs, line))
    {
        class_list.push_back(line);
    }
    return class_list;
}

void load_net(cv::dnn::Net &net, const std::string &net_path, bool is_cuda)
{
    auto result = cv::dnn::readNet(net_path);
    if (is_cuda)
    {
        ROS_INFO("Attempting to use CUDA\n");
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        ROS_INFO("success setPreferableBackend");
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
        ROS_INFO("success setPreferableTarget");
    }
    else
    {
        ROS_INFO("Running on CPU\n");
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    net = result;
}

const std::vector<cv::Scalar> colors = {cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0)};

const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.4;
const float CONFIDENCE_THRESHOLD = 0.5;

struct Detection
{
    int class_id;
    float confidence;
    cv::Rect box;
};

cv::Mat format_yolov5(const cv::Mat &source) {
    int col = source.cols;
    int row = source.rows;
    int _max = std::max(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

void detect(cv::Mat &image, cv::dnn::Net &net, std::vector<Detection> &output, const std::vector<std::string> &className) {
    cv::Mat blob;

    auto input_image = format_yolov5(image);
    ROS_INFO("successfully transform the img");
    
    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
    ROS_INFO("FINISH blobFromImage");
    net.setInput(blob);
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    ROS_INFO("SUCCESS forward");

    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;
    
    float *data = (float *)outputs[0].data;

    const int dimensions = 9;
    const int rows = 25200;
    
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    // FIXME 成功打印出forward了，问题出在这个部分！
    for (int i = 0; i < rows; ++i) {
        float confidence = data[4];
        ROS_INFO("computed confidence");
        if (confidence >= CONFIDENCE_THRESHOLD) {
            float * classes_scores = data + 5;
            ROS_INFO("initialise classes_scores");
            cv::Mat scores(1, className.size(), CV_32FC1, classes_scores);
            ROS_INFO("initialise scores");
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            ROS_INFO("minMaxLoc");
            if (max_class_score > SCORE_THRESHOLD) {
                ROS_INFO("entern max_class_score>SCORE_THRESHOLD");
                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
        data += 9;
    }
    ROS_INFO("finish first for loop (rows) in detect function");

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
    ROS_INFO("initialize NMSBoxes");
    for (int i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);
    }
}

void image_cb(const sensor_msgs::ImageConstPtr &msg, cv::dnn::Net &net, const std::vector<std::string> &class_list, ros::Publisher &bbox_pub, image_transport::Publisher &image_pub) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    ROS_INFO("successfully get the frame");
    std::vector<Detection> output;
    detect(frame, net, output, class_list);
    ROS_INFO("successfully detect the frame");

    zal_yolov5::BoundingBoxes bbox_msg;
    bbox_msg.header = msg->header;

    int detections = output.size();
    for (int i = 0; i < detections; ++i) {
        auto detection = output[i];
        auto box = detection.box;
        auto classId = detection.class_id;

        zal_yolov5::BoundingBox bbox;
        bbox.Class = class_list[detection.class_id];
        bbox.probability = detection.confidence;
        bbox.xmin = box.x;
        bbox.ymin = box.y;
        bbox.xmax = box.x + box.width;
        bbox.ymax = box.y + box.height;
        bbox_msg.bounding_boxes.push_back(bbox);

        const auto color = colors[classId % colors.size()];
        cv::rectangle(frame, box, color, 3);

        cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
        cv::putText(frame, class_list[classId].c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
    ROS_INFO("successfully generated the bbxs and drawing bbxs");
    bbox_pub.publish(bbox_msg);
    ROS_INFO("success pub bbx");

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    image_pub.publish(img_msg);
    ROS_INFO("success pub annotated img");
}

int main(int argc, char **argv)
{
    ROS_INFO("start node zal_yolov5");
    ros::init(argc, argv, "zal_yolov5_node");
    ros::NodeHandle nh;

    std::string net_path, class_list_path;
    bool use_cuda = true;

    net_path = "/home/nvidia/zal_ws/src/zal_yolov5/config/best.onnx";
    class_list_path = "/home/nvidia/zal_ws/src/zal_yolov5/config/classes.txt";

    // Get parameters from the parameter server
    nh.getParam("net_path", net_path);
    nh.getParam("class_list_path", class_list_path);
    nh.getParam("use_cuda", use_cuda);

    ROS_INFO("net_path: %s", net_path.c_str());
    ROS_INFO("class_list_path: %s", class_list_path.c_str());

    // // Resolve relative paths
    // if (net_path[0] != '/')
    // {
    //     net_path = ros::package::getPath("zal_yolov5") + "/" + net_path;
    // }
    // if (class_list_path[0] != '/')
    // {
    //     class_list_path = ros::package::getPath("zal_yolov5") + "/" + class_list_path;
    // }

    std::vector<std::string> class_list = load_class_list(class_list_path);

    cv::dnn::Net net;
    load_net(net, net_path, use_cuda);

    ROS_INFO("finish load_net");

    image_transport::ImageTransport it(nh);
    ROS_INFO("initialize image_transport");
    image_transport::Publisher image_pub = it.advertise("detected_image", 1);
    ROS_INFO("initialize image_pub");
    ros::Publisher bbox_pub = nh.advertise<zal_yolov5::BoundingBoxes>("bounding_boxes", 1);
    ROS_INFO("initialize bbox_pub");

    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1,
        boost::bind(image_cb, _1, boost::ref(net), boost::ref(class_list), boost::ref(bbox_pub), boost::ref(image_pub)));

    ros::spin();

    return 0;
}

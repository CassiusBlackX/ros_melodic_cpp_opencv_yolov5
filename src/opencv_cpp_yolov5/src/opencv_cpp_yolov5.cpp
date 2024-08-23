#include <fstream>
#include <string>
#include <filesystem>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Header.h>
#include <opencv_cpp_yolov5/BoundingBox.h>
#include <opencv_cpp_yolov5/BoundingBoxes.h>

namespace fs = std::filesystem;

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

void load_yolo(cv::dnn::Net &yolo, const std::string &net_path, bool is_cuda)
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
    yolo = result;
}

void load_resnet(cv::dnn::Net &net, const std::string &net_path, bool is_cuda) {
    auto result = cv::dnn::readNetFromONNX(net_path);
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
float SCORE_THRESHOLD = 0.7;
float NMS_THRESHOLD = 0.4;
float CONFIDENCE_THRESHOLD = 0.79;

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

void detect(cv::Mat &image, cv::dnn::Net &yolo, cv::dnn::Net &resnet, std::vector<Detection> &output, const std::vector<std::string> &className) {
    cv::Mat blob;

    auto input_image = format_yolov5(image);
    
    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
    yolo.setInput(blob);
    std::vector<cv::Mat> outputs;
    yolo.forward(outputs, yolo.getUnconnectedOutLayersNames());

    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;
    
    float *data = (float *)outputs[0].data;

    const int dimensions = 9;  // dimensions = class_num + 5
    const int rows = 25200;
    
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i) {
        float confidence = data[4];
        if (confidence >= CONFIDENCE_THRESHOLD) {
            float * classes_scores = data + 5;
            cv::Mat scores(1, className.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            if (max_class_score > SCORE_THRESHOLD) {
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
        data += dimensions;
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
    for (int i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);

        // crop bbx and use resnet to classify again, for higher accuracy
        cv::Mat crop = image(result.box).clone();
        cv::resize(crop, crop, cv::Size(224, 224));  // ResNet input is 224*224
        cv::Mat reset_blob;
        cv::dnn::blobFromImage(crop, reset_blob, 1./255., cv::Size(224, 224), cv::Scalar(), true, false);  //BUG not sure about the 3rd parameter
        resnet.setInput(reset_blob);
        cv::Mat resnet_output = resnet.forward();  
        // resnet.forward(resnet_output, resnet.getUnconnectedOutLayersNames());

        // get the class id
        cv::Point class_id;
        double max_class_score;
        minMaxLoc(resnet_output, 0, &max_class_score, 0, &class_id);
        ROS_INFO("class_id: %d", class_id.x);
        ROS_INFO("max_class_score: %f", max_class_score);

        // judge result from yolo and resnet
        // TODO
    }
}

void image_cb(const sensor_msgs::ImageConstPtr &msg, cv::dnn::Net &yolo, const std::vector<std::string> &class_list, ros::Publisher &bbox_pub, image_transport::Publisher &image_pub) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    std::vector<Detection> output;
    detect(frame, yolo, output, class_list);

    opencv_cpp_yolov5::BoundingBoxes bbox_msg;
    bbox_msg.header = msg->header;

    int detections = output.size();
    for (int i = 0; i < detections; ++i) {
        auto detection = output[i];
        auto box = detection.box;
        auto classId = detection.class_id;

        opencv_cpp_yolov5::BoundingBox bbox;
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
    bbox_pub.publish(bbox_msg);

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    image_pub.publish(img_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_cpp_yolov5_node");
    
    ROS_WARN("start node opencv_cpp_yolov5");
    ros::NodeHandle nh("~");

    std::string net_path, class_list_path;
    bool use_cuda = true;

    // Get parameters from the parameter server
    nh.getParam("net_path", net_path);
    nh.getParam("class_list_path", class_list_path);
    nh.getParam("use_cuda", use_cuda);
    nh.getParam("score_threshold", SCORE_THRESHOLD);
    nh.getParam("nms_threshold", NMS_THRESHOLD);
    nh.getParam("confidence_threshold", CONFIDENCE_THRESHOLD);

    // net_path = "/home/nvidia/zal_ws/src/opencv_cpp_yolov5/config/best.onnx";
    // class_list_path = "/home/nvidia/zal_ws/src/opencv_cpp_yolov5/config/classes.txt";

    ROS_INFO("net_path: %s", net_path.c_str());
    ROS_WARN("net_path: %s", net_path.c_str());
    ROS_INFO("class_list_path: %s", class_list_path.c_str());
    ROS_WARN("class_list_path: %s", class_list_path.c_str());
    ROS_WARN("score_threshold: %f", SCORE_THRESHOLD);
    ROS_WARN("nms_threshold: %f", NMS_THRESHOLD);
    ROS_WARN("confidence_threshold: %f", CONFIDENCE_THRESHOLD);

    std::vector<std::string> class_list = load_class_list(class_list_path);

    cv::dnn::Net yolo;
    load_yolo(yolo, net_path, use_cuda);

    ROS_INFO("finish load_yolo");

    image_transport::ImageTransport it(nh);
    ROS_INFO("initialize image_transport");
    image_transport::Publisher image_pub = it.advertise("/opencv_cpp_yolov5/detected_image", 1);
    ROS_INFO("initialize image_pub");
    ros::Publisher bbox_pub = nh.advertise<opencv_cpp_yolov5::BoundingBoxes>("/opencv_cpp_yolov5/bounding_boxes", 1);
    ROS_INFO("initialize bbox_pub");

    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1,
        boost::bind(image_cb, _1, boost::ref(yolo), boost::ref(class_list), boost::ref(bbox_pub), boost::ref(image_pub)));

    ros::spin();

    return 0;
}

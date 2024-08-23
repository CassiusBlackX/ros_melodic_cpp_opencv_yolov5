// #define USE_RESNET

#include <fstream>
#include <string>
#include <chrono>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <log4cxx/logger.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Header.h>
#ifdef USE_RESNET
#include <opencv_cpp_yolov5/BoundingBoxResNet.h>
#include <opencv_cpp_yolov5/BoundingBoxesResNet.h>
#define BoundingBox BoundingBoxResNet
#define BoundingBoxes BoundingBoxesResNet
#else
#include <opencv_cpp_yolov5/BoundingBox.h>
#include <opencv_cpp_yolov5/BoundingBoxes.h>
#endif

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

#ifdef USE_RESNET
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
#endif

const std::vector<cv::Scalar> boundingBoxColors = {cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0)};

const float YOLO_INPUT_WIDTH = 640.0;
const float YOLO_INPUT_HEIGHT = 640.0;

#ifdef USE_RESNET
const float RESNET_INPUT_WIDTH = 224.0;
const float RESNET_INPUT_HEIGHT = 224.0;
#endif

float NMS_THRESHOLD = 0.4;
float CONFIDENCE_THRESHOLD = 0.79;  // filter out boxes with confidence lower than this
float SCORE_THRESHOLD = 0.8;  // filter out classes with score lower than this

struct Detection
{
    int class_id;
    float confidence;
    cv::Rect box;
    #ifdef USE_RESNET
    int resnet_class_id;
    float resnet_confidence;
    #endif
};

cv::Mat format_yolov5(const cv::Mat &source) {
    int col = source.cols;
    int row = source.rows;
    int _max = std::max(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

void detect(cv::Mat &image, cv::dnn::Net &yolo, std::vector<Detection> &output, const std::vector<std::string> &className
#ifdef USE_RESNET
, cv::dnn::Net &resnet
#endif
) {
    cv::Mat blob;

    auto input_image = format_yolov5(image);
    
    cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(YOLO_INPUT_WIDTH, YOLO_INPUT_HEIGHT), cv::Scalar(), true, false);
    yolo.setInput(blob);
    std::vector<cv::Mat> outputs;
    yolo.forward(outputs, yolo.getUnconnectedOutLayersNames());

    float x_factor = input_image.cols / YOLO_INPUT_WIDTH;
    float y_factor = input_image.rows / YOLO_INPUT_HEIGHT;
    
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

        #ifdef USE_RESNET
        // crop bbx and use resnet to classify again, for higher accuracy
        cv::Mat crop = image(result.box).clone();
        cv::resize(crop, crop, cv::Size(RESNET_INPUT_WIDTH, RESNET_INPUT_HEIGHT));  // ResNet input is 224*224
        cv::Mat resnet_blob;
        cv::dnn::blobFromImage(crop, resnet_blob, 1./255., cv::Size(RESNET_INPUT_WIDTH, RESNET_INPUT_HEIGHT), cv::Scalar(), true, false);
        resnet.setInput(resnet_blob);
        cv::Mat resnet_output = resnet.forward();  
        // resnet.forward(resnet_output, resnet.getUnconnectedOutLayersNames());

        // get the class id
        cv::Point resnet_class_id;
        double max_class_score;
        minMaxLoc(resnet_output, 0, &max_class_score, 0, &resnet_class_id);

        if (resnet_class_id.x != result.class_id) {
            ROS_WARN("resnet_class_id: %d, yolo.class_id: %d", resnet_class_id.x, result.class_id);
            ROS_WARN("resnet_confidence: %f, yolo_confidence: %f", max_class_score, result.confidence);
        } 
        result.resnet_class_id = resnet_class_id.x;
        result.resnet_confidence = max_class_score;
       #endif
        
        output.push_back(result);
    }
}

void image_cb(const sensor_msgs::ImageConstPtr &msg, cv::dnn::Net &yolo, const std::vector<std::string> &class_list, ros::Publisher &bbox_pub, image_transport::Publisher &image_pub
#ifdef USE_RESNET
, cv::dnn::Net &resnet
#endif
) {
    // record time
    auto start_time = std::chrono::steady_clock::now();

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    std::vector<Detection> output;
    detect(frame, yolo, output, class_list
    #ifdef USE_RESNET
    , resnet
    #endif
    );

    opencv_cpp_yolov5::BoundingBoxes bbox_msg;
    bbox_msg.header = msg->header;

    int detections = output.size();
    for (int i = 0; i < detections; ++i) {
        auto detection = output[i];
        auto box = detection.box;
        auto classId = detection.class_id;
        #ifdef USE_RESNET
        auto resnet_classId = detection.resnet_class_id;
        auto resnet_confidence = detection.resnet_confidence;
        #endif

        opencv_cpp_yolov5::BoundingBox bbox;
        bbox.Class = class_list[detection.class_id];
        #ifdef USE_RESNET
        bbox.resnet_class = class_list[detection.resnet_class_id];
        bbox.resnet_probability = detection.resnet_confidence;
        #endif
        bbox.probability = detection.confidence;
        bbox.xmin = box.x;
        bbox.ymin = box.y;
        bbox.xmax = box.x + box.width;
        bbox.ymax = box.y + box.height;
        bbox_msg.bounding_boxes.push_back(bbox);

        const auto color = boundingBoxColors[classId % boundingBoxColors.size()];
        cv::rectangle(frame, box, color, 3);

        cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
        std::string label;
        #ifdef USE_RESNET
        // label = (classId == resnet_classId) ? (class_list[classId] + " (" + std::to_string(detection.confidence) + ")") : ("YOlO: " + class_list[classId] + " (" + std::to_string(detection.confidence) + ") ResNet: " + class_list[resnet_classId] + " (" + std::to_string(detection.resnet_confidence) + ")");
        label = "YOlO: " + class_list[classId] + " (" + std::to_string(detection.confidence) + ")\n ResNet: " + class_list[resnet_classId] + " (" + std::to_string(detection.resnet_confidence) + ")";
        #else
        label = class_list[classId] + " (" + std::to_string(detection.confidence) + ")";
        #endif
        cv::putText(frame, label.c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
    bbox_pub.publish(bbox_msg);

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    image_pub.publish(img_msg);

    // record time
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;
    float fps = 1.0 / elapsed_time.count();
    ROS_WARN("FPS: %f", fps);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_cpp_yolov5_node");
    
    ROS_INFO("start node opencv_cpp_yolov5");
    ros::NodeHandle nh("~");

    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
    //     ros::console::notifyLoggerLevelsChanged();
    // }

    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
    ros::console::notifyLoggerLevelsChanged();

    std::string yolo_path, resnet_path, class_list_path;
    bool use_cuda = true;

    // Get parameters from the parameter server
    nh.getParam("yolo_path", yolo_path);
    #ifdef USE_RESNET
    nh.getParam("resnet_path", resnet_path);
    #endif
    nh.getParam("class_list_path", class_list_path);
    nh.getParam("use_cuda", use_cuda);
    nh.getParam("score_threshold", SCORE_THRESHOLD);
    nh.getParam("nms_threshold", NMS_THRESHOLD);
    nh.getParam("confidence_threshold", CONFIDENCE_THRESHOLD);

    ROS_INFO("yolo_path: %s", yolo_path.c_str());
    #ifdef USE_RESNET
    ROS_WARN("resnet_path: %s", resnet_path.c_str());
    #endif
    ROS_INFO("class_list_path: %s", class_list_path.c_str());
    ROS_WARN("class_list_path: %s", class_list_path.c_str());
    ROS_WARN("score_threshold: %f", SCORE_THRESHOLD);
    ROS_WARN("nms_threshold: %f", NMS_THRESHOLD);
    ROS_WARN("confidence_threshold: %f", CONFIDENCE_THRESHOLD);

    std::vector<std::string> class_list = load_class_list(class_list_path);

    cv::dnn::Net yolo;
    load_yolo(yolo, yolo_path, use_cuda);
    #ifdef USE_RESNET
    cv::dnn::Net resnet;
    load_resnet(resnet, resnet_path, use_cuda);
    #endif

    ROS_INFO("finish load_yolo");

    image_transport::ImageTransport it(nh);
    ROS_INFO("initialize image_transport");
    image_transport::Publisher image_pub = it.advertise("/opencv_cpp_yolov5/detected_image", 1);
    ROS_INFO("initialize image_pub");
    ros::Publisher bbox_pub = nh.advertise<opencv_cpp_yolov5::BoundingBoxes>("/opencv_cpp_yolov5/bounding_boxes", 1);
    ROS_INFO("initialize bbox_pub");

    image_transport::Subscriber sub = it.subscribe("/video_stream_node/image_raw", 1,
        boost::bind(image_cb, _1, boost::ref(yolo), boost::ref(class_list), boost::ref(bbox_pub), boost::ref(image_pub)
        #ifdef USE_RESNET
        , boost::ref(resnet)
        #endif
        ));

    ros::spin();

    return 0;
}

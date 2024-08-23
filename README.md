# ros_melodic_cpp_opencv_yolov5
this is a catkin workspace which built for visual detection, originally for ros melodic, but can also work on ros noetic with a little modification.

## cunstruction
+ `vision_opencv` is the official package for `cv_bridge` compatible, and should not be changed
+ `opencv_cpp_yolov5` is the customed package
  + `src/opencv_cpp_yolov5 node` is the node subscribing images from usb_cam and use yolov5 to detect the image
  + `src/hough_circle_detector` is the node subscribing images from usb_cam and use hough_circle to find the most likely circle in the image
  + `config` is the directory where the yolov5 weights file `*.onnx` and `classes.txt` are placed
    + `rosconsole.config` is the file i used to set ros logger, but failed in my own env.
  + `shell` includes a script to auto start all the nodes needed at a time

## subscribe messages
+ `ubs_cam/image_raw` is the topic advertised by the melodic default `usb_cam.launch` 
+ (optional) `/video_stream/image_raw` is the message published by `video_stream_node`, who will open a video in the given path and publishing it over and over again

## published meassages
+ `opencv_cpp_yolov5/detected_image`  the annotated image by yolov5
+ `opencv_cpp_yolov5/bounding_boxes` the bbxs by yolov5, based on messages from [`darknet_ros`](https://github.com/leggedrobotics/darknet_ros)
+ `opencv_cpp_yolov5/circle_detect_result_img` is the annotated image by hough_circle
+ `opencv_cpp_yolov5/circle_detect_result` is the circle info by hough_circle

## NOTE
+ it is highly recommended to manually `catkin_create_workspace` to create a workspace, then copy the `src` directory into the workspace, and finally `catkin_make`. Or else there is chance that when trying to `find_package` in other workspace, `cmake` would fail to find `opencv_cpp_yolov5` even with the right environment
+ for ros noetic, you should modify `vision_opencv/cv_bridge/src/module.hpp` to its original form

```cpp
static void * do_numpy_import( )  // uncomment this line
// static void do_numpy_import( )  // comment or delete this line
{
    import_array( );
    return nullptr;  // uncomment this line
}
```

## new features
in `opencv_cpp_yolov5/src/opencv_cpp_yolov5.cpp`, there is an macro named `USE_RESNET`, if the macro is defined, you should pass in the onnx form of resnet50 weight path(recommended) or other model weight. and before publishing messages, we will compare the inference results by YOLO and resnet, where resnet uses the cropped image from YOLO.

in this way, when YOLO captured a bounding box but give out an incorrect classification, resnet will give another answer, and you should determine whose answer you are going to adopt.
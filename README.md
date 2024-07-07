# ros_melodic_cpp_opencv_yolov5
this is a catkin workspace which built for visual detection.

## cunstruction
+ `vision_opencv` is the official package for `cv_bridge` compatible, and should not be changed
+ `opencv_cpp_yolov5` is the customed package
  + `src/opencv_cpp_yolov5 node` is the node subscribing images from usb_cam and use yolov5 to detect the image
  + `src/hough_circle_detector` is the node subscribing images from usb_cam and use hough_circle to find the most likely circle in the image
  + `config` is the directory where the yolov5 weights file `*.onnx` and `classes.txt` are placed
  + `shell` includes a script to auto start all the nodes needed at a time

## subscribe messages
+ `ubs_cam/image_raw` is the topic advertised by the melodic default `usb_cam.launch` 

## published meassages
+ `opencv_cpp_yolov5/detected_image`  the annotated image by yolov5
+ `opencv_cpp_yolov5/bounding_boxes` the bbxs by yolov5, based on messages from [`darknet_ros`](https://github.com/leggedrobotics/darknet_ros)
+ `opencv_cpp_yolov5/circle_detect_result_img` is the annotated image by hough_circle
+ `opencv_cpp_yolov5/circle_detect_result` is the circle info by hough_circle


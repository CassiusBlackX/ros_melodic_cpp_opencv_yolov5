;; Auto-generated. Do not edit!


(when (boundp 'zal_yolov5::BoundingBoxes)
  (if (not (find-package "ZAL_YOLOV5"))
    (make-package "ZAL_YOLOV5"))
  (shadow 'BoundingBoxes (find-package "ZAL_YOLOV5")))
(unless (find-package "ZAL_YOLOV5::BOUNDINGBOXES")
  (make-package "ZAL_YOLOV5::BOUNDINGBOXES"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxes.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass zal_yolov5::BoundingBoxes
  :super ros::object
  :slots (_header _bounding_boxes ))

(defmethod zal_yolov5::BoundingBoxes
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:bounding_boxes __bounding_boxes) (let (r) (dotimes (i 0) (push (instance zal_yolov5::BoundingBox :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _bounding_boxes __bounding_boxes)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:bounding_boxes
   (&rest __bounding_boxes)
   (if (keywordp (car __bounding_boxes))
       (send* _bounding_boxes __bounding_boxes)
     (progn
       (if __bounding_boxes (setq _bounding_boxes (car __bounding_boxes)))
       _bounding_boxes)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; zal_yolov5/BoundingBox[] _bounding_boxes
    (apply #'+ (send-all _bounding_boxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; zal_yolov5/BoundingBox[] _bounding_boxes
     (write-long (length _bounding_boxes) s)
     (dolist (elem _bounding_boxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; zal_yolov5/BoundingBox[] _bounding_boxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bounding_boxes (let (r) (dotimes (i n) (push (instance zal_yolov5::BoundingBox :init) r)) r))
     (dolist (elem- _bounding_boxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get zal_yolov5::BoundingBoxes :md5sum-) "0c66b76f3c0a60b31a7bca182a3b5d0d")
(setf (get zal_yolov5::BoundingBoxes :datatype-) "zal_yolov5/BoundingBoxes")
(setf (get zal_yolov5::BoundingBoxes :definition-)
      "Header header
BoundingBox[] bounding_boxes
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: zal_yolov5/BoundingBox
float64 probability
int64 xmin
int64 ymin
int64 xmax
int64 ymax
string Class
")



(provide :zal_yolov5/BoundingBoxes "0c66b76f3c0a60b31a7bca182a3b5d0d")



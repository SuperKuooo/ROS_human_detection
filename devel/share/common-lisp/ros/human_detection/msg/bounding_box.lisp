; Auto-generated. Do not edit!


(cl:in-package human_detection-msg)


;//! \htmlinclude bounding_box.msg.html

(cl:defclass <bounding_box> (roslisp-msg-protocol:ros-message)
  ((xmin
    :reader xmin
    :initarg :xmin
    :type cl:fixnum
    :initform 0)
   (xmax
    :reader xmax
    :initarg :xmax
    :type cl:fixnum
    :initform 0)
   (ymin
    :reader ymin
    :initarg :ymin
    :type cl:fixnum
    :initform 0)
   (ymax
    :reader ymax
    :initarg :ymax
    :type cl:fixnum
    :initform 0))
)

(cl:defclass bounding_box (<bounding_box>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bounding_box>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bounding_box)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name human_detection-msg:<bounding_box> is deprecated: use human_detection-msg:bounding_box instead.")))

(cl:ensure-generic-function 'xmin-val :lambda-list '(m))
(cl:defmethod xmin-val ((m <bounding_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_detection-msg:xmin-val is deprecated.  Use human_detection-msg:xmin instead.")
  (xmin m))

(cl:ensure-generic-function 'xmax-val :lambda-list '(m))
(cl:defmethod xmax-val ((m <bounding_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_detection-msg:xmax-val is deprecated.  Use human_detection-msg:xmax instead.")
  (xmax m))

(cl:ensure-generic-function 'ymin-val :lambda-list '(m))
(cl:defmethod ymin-val ((m <bounding_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_detection-msg:ymin-val is deprecated.  Use human_detection-msg:ymin instead.")
  (ymin m))

(cl:ensure-generic-function 'ymax-val :lambda-list '(m))
(cl:defmethod ymax-val ((m <bounding_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader human_detection-msg:ymax-val is deprecated.  Use human_detection-msg:ymax instead.")
  (ymax m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bounding_box>) ostream)
  "Serializes a message object of type '<bounding_box>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'xmin)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'xmin)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'xmax)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'xmax)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ymin)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ymin)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ymax)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ymax)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bounding_box>) istream)
  "Deserializes a message object of type '<bounding_box>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'xmin)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'xmin)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'xmax)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'xmax)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ymin)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ymin)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ymax)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ymax)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bounding_box>)))
  "Returns string type for a message object of type '<bounding_box>"
  "human_detection/bounding_box")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bounding_box)))
  "Returns string type for a message object of type 'bounding_box"
  "human_detection/bounding_box")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bounding_box>)))
  "Returns md5sum for a message object of type '<bounding_box>"
  "0a18150eb9bc571abec460d2df647248")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bounding_box)))
  "Returns md5sum for a message object of type 'bounding_box"
  "0a18150eb9bc571abec460d2df647248")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bounding_box>)))
  "Returns full string definition for message of type '<bounding_box>"
  (cl:format cl:nil "uint16 xmin~%uint16 xmax~%uint16 ymin~%uint16 ymax~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bounding_box)))
  "Returns full string definition for message of type 'bounding_box"
  (cl:format cl:nil "uint16 xmin~%uint16 xmax~%uint16 ymin~%uint16 ymax~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bounding_box>))
  (cl:+ 0
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bounding_box>))
  "Converts a ROS message object to a list"
  (cl:list 'bounding_box
    (cl:cons ':xmin (xmin msg))
    (cl:cons ':xmax (xmax msg))
    (cl:cons ':ymin (ymin msg))
    (cl:cons ':ymax (ymax msg))
))

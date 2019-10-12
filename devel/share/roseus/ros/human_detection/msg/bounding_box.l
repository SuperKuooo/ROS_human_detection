;; Auto-generated. Do not edit!


(when (boundp 'human_detection::bounding_box)
  (if (not (find-package "HUMAN_DETECTION"))
    (make-package "HUMAN_DETECTION"))
  (shadow 'bounding_box (find-package "HUMAN_DETECTION")))
(unless (find-package "HUMAN_DETECTION::BOUNDING_BOX")
  (make-package "HUMAN_DETECTION::BOUNDING_BOX"))

(in-package "ROS")
;;//! \htmlinclude bounding_box.msg.html


(defclass human_detection::bounding_box
  :super ros::object
  :slots (_xmin _xmax _ymin _ymax ))

(defmethod human_detection::bounding_box
  (:init
   (&key
    ((:xmin __xmin) 0)
    ((:xmax __xmax) 0)
    ((:ymin __ymin) 0)
    ((:ymax __ymax) 0)
    )
   (send-super :init)
   (setq _xmin (round __xmin))
   (setq _xmax (round __xmax))
   (setq _ymin (round __ymin))
   (setq _ymax (round __ymax))
   self)
  (:xmin
   (&optional __xmin)
   (if __xmin (setq _xmin __xmin)) _xmin)
  (:xmax
   (&optional __xmax)
   (if __xmax (setq _xmax __xmax)) _xmax)
  (:ymin
   (&optional __ymin)
   (if __ymin (setq _ymin __ymin)) _ymin)
  (:ymax
   (&optional __ymax)
   (if __ymax (setq _ymax __ymax)) _ymax)
  (:serialization-length
   ()
   (+
    ;; uint16 _xmin
    2
    ;; uint16 _xmax
    2
    ;; uint16 _ymin
    2
    ;; uint16 _ymax
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _xmin
       (write-word _xmin s)
     ;; uint16 _xmax
       (write-word _xmax s)
     ;; uint16 _ymin
       (write-word _ymin s)
     ;; uint16 _ymax
       (write-word _ymax s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _xmin
     (setq _xmin (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _xmax
     (setq _xmax (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _ymin
     (setq _ymin (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _ymax
     (setq _ymax (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get human_detection::bounding_box :md5sum-) "0a18150eb9bc571abec460d2df647248")
(setf (get human_detection::bounding_box :datatype-) "human_detection/bounding_box")
(setf (get human_detection::bounding_box :definition-)
      "uint16 xmin
uint16 xmax
uint16 ymin
uint16 ymax

")



(provide :human_detection/bounding_box "0a18150eb9bc571abec460d2df647248")



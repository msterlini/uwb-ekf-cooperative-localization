;; Auto-generated. Do not edit!


(when (boundp 'dist_project::robot_data)
  (if (not (find-package "DIST_PROJECT"))
    (make-package "DIST_PROJECT"))
  (shadow 'robot_data (find-package "DIST_PROJECT")))
(unless (find-package "DIST_PROJECT::ROBOT_DATA")
  (make-package "DIST_PROJECT::ROBOT_DATA"))

(in-package "ROS")
;;//! \htmlinclude robot_data.msg.html


(defclass dist_project::robot_data
  :super ros::object
  :slots (_robot_id _target_height _x _y _theta _sigma_x _sigma_y _sigma_theta ))

(defmethod dist_project::robot_data
  (:init
   (&key
    ((:robot_id __robot_id) "")
    ((:target_height __target_height) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    ((:sigma_x __sigma_x) 0.0)
    ((:sigma_y __sigma_y) 0.0)
    ((:sigma_theta __sigma_theta) 0.0)
    )
   (send-super :init)
   (setq _robot_id (string __robot_id))
   (setq _target_height (float __target_height))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   (setq _sigma_x (float __sigma_x))
   (setq _sigma_y (float __sigma_y))
   (setq _sigma_theta (float __sigma_theta))
   self)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:target_height
   (&optional __target_height)
   (if __target_height (setq _target_height __target_height)) _target_height)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:sigma_x
   (&optional __sigma_x)
   (if __sigma_x (setq _sigma_x __sigma_x)) _sigma_x)
  (:sigma_y
   (&optional __sigma_y)
   (if __sigma_y (setq _sigma_y __sigma_y)) _sigma_y)
  (:sigma_theta
   (&optional __sigma_theta)
   (if __sigma_theta (setq _sigma_theta __sigma_theta)) _sigma_theta)
  (:serialization-length
   ()
   (+
    ;; string _robot_id
    4 (length _robot_id)
    ;; float64 _target_height
    8
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _theta
    8
    ;; float64 _sigma_x
    8
    ;; float64 _sigma_y
    8
    ;; float64 _sigma_theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _robot_id
       (write-long (length _robot_id) s) (princ _robot_id s)
     ;; float64 _target_height
       (sys::poke _target_height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _sigma_x
       (sys::poke _sigma_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _sigma_y
       (sys::poke _sigma_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _sigma_theta
       (sys::poke _sigma_theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _robot_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _target_height
     (setq _target_height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _sigma_x
     (setq _sigma_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _sigma_y
     (setq _sigma_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _sigma_theta
     (setq _sigma_theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get dist_project::robot_data :md5sum-) "de74b7f6fa08eefd9305c86ead89bd1b")
(setf (get dist_project::robot_data :datatype-) "dist_project/robot_data")
(setf (get dist_project::robot_data :definition-)
      "string robot_id
float64 target_height
float64 x
float64 y
float64 theta
float64 sigma_x
float64 sigma_y
float64 sigma_theta
")



(provide :dist_project/robot_data "de74b7f6fa08eefd9305c86ead89bd1b")



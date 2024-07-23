;; Auto-generated. Do not edit!


(when (boundp 'dist_project::uwb_data)
  (if (not (find-package "DIST_PROJECT"))
    (make-package "DIST_PROJECT"))
  (shadow 'uwb_data (find-package "DIST_PROJECT")))
(unless (find-package "DIST_PROJECT::UWB_DATA")
  (make-package "DIST_PROJECT::UWB_DATA"))

(in-package "ROS")
;;//! \htmlinclude uwb_data.msg.html


(defclass dist_project::uwb_data
  :super ros::object
  :slots (_tag_id _distance ))

(defmethod dist_project::uwb_data
  (:init
   (&key
    ((:tag_id __tag_id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:distance __distance) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _tag_id __tag_id)
   (setq _distance __distance)
   self)
  (:tag_id
   (&optional __tag_id)
   (if __tag_id (setq _tag_id __tag_id)) _tag_id)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; int64[] _tag_id
    (* 8    (length _tag_id)) 4
    ;; float64[] _distance
    (* 8    (length _distance)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64[] _tag_id
     (write-long (length _tag_id) s)
     (dotimes (i (length _tag_id))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _tag_id i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _tag_id i)) (= (length ((elt _tag_id i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _tag_id i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _tag_id i) . bv) 1) -1) s))
             ((and (class (elt _tag_id i)) (= (length ((elt _tag_id i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _tag_id i) . bv) 0) s)
              (write-long (if (>= (elt _tag_id i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _tag_id i) s)(write-long (if (>= (elt _tag_id i) 0) 0 #xffffffff) s)))
       )
     ;; float64[] _distance
     (write-long (length _distance) s)
     (dotimes (i (length _distance))
       (sys::poke (elt _distance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64[] _tag_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tag_id (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _tag_id i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _tag_id i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; float64[] _distance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _distance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _distance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get dist_project::uwb_data :md5sum-) "17e78c3780628319859d2345ed3e02ce")
(setf (get dist_project::uwb_data :datatype-) "dist_project/uwb_data")
(setf (get dist_project::uwb_data :definition-)
      "int64[] tag_id
float64[] distance
")



(provide :dist_project/uwb_data "17e78c3780628319859d2345ed3e02ce")



;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::Num)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'Num (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::NUM")
  (make-package "BEGINNER_TUTORIALS::NUM"))

(in-package "ROS")
;;//! \htmlinclude Num.msg.html


(defclass beginner_tutorials::Num
  :super ros::object
  :slots (_Num ))

(defmethod beginner_tutorials::Num
  (:init
   (&key
    ((:Num __Num) 0)
    )
   (send-super :init)
   (setq _Num (round __Num))
   self)
  (:Num
   (&optional __Num)
   (if __Num (setq _Num __Num)) _Num)
  (:serialization-length
   ()
   (+
    ;; int64 _Num
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _Num
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _Num (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _Num) (= (length (_Num . bv)) 2)) ;; bignum
              (write-long (ash (elt (_Num . bv) 0) 0) s)
              (write-long (ash (elt (_Num . bv) 1) -1) s))
             ((and (class _Num) (= (length (_Num . bv)) 1)) ;; big1
              (write-long (elt (_Num . bv) 0) s)
              (write-long (if (>= _Num 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _Num s)(write-long (if (>= _Num 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _Num
#+(or :alpha :irix6 :x86_64)
      (setf _Num (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _Num (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get beginner_tutorials::Num :md5sum-) "94656e1b849ade935c34cb79b08be4df")
(setf (get beginner_tutorials::Num :datatype-) "beginner_tutorials/Num")
(setf (get beginner_tutorials::Num :definition-)
      "int64 Num
")



(provide :beginner_tutorials/Num "94656e1b849ade935c34cb79b08be4df")



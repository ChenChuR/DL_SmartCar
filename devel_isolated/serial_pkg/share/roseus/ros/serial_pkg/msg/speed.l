;; Auto-generated. Do not edit!


(when (boundp 'serial_pkg::speed)
  (if (not (find-package "SERIAL_PKG"))
    (make-package "SERIAL_PKG"))
  (shadow 'speed (find-package "SERIAL_PKG")))
(unless (find-package "SERIAL_PKG::SPEED")
  (make-package "SERIAL_PKG::SPEED"))

(in-package "ROS")
;;//! \htmlinclude speed.msg.html


(defclass serial_pkg::speed
  :super ros::object
  :slots (_vx _angle _signage ))

(defmethod serial_pkg::speed
  (:init
   (&key
    ((:vx __vx) 0.0)
    ((:angle __angle) 0.0)
    ((:signage __signage) 0)
    )
   (send-super :init)
   (setq _vx (float __vx))
   (setq _angle (float __angle))
   (setq _signage (round __signage))
   self)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:signage
   (&optional __signage)
   (if __signage (setq _signage __signage)) _signage)
  (:serialization-length
   ()
   (+
    ;; float32 _vx
    4
    ;; float32 _angle
    4
    ;; uint8 _signage
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _signage
       (write-byte _signage s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _vx
     (setq _vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _signage
     (setq _signage (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get serial_pkg::speed :md5sum-) "628d09100e1102fef4349bb734fbb743")
(setf (get serial_pkg::speed :datatype-) "serial_pkg/speed")
(setf (get serial_pkg::speed :definition-)
      "float32 vx
float32 angle
uint8 signage
")



(provide :serial_pkg/speed "628d09100e1102fef4349bb734fbb743")



#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")

(defun setup (&optional (real nil))
  (load "package://hsreus/hsrb-interface.l")
  (hsrb-init)
  (setq *robot* *hsrb*)
  (objects (list *robot*))
  )

(defun move-to-spot (spot)
  (let (target)
    (when (null (send *tfl* :wait-for-transform "/map" spot (ros::time 0) 1))
      (return-from move-to-spot nil))
    (setq target (send *tfl* :lookup-transform "/map" spot (ros::time 0)))
    (when (null target)
      (return-from move-to-spot nil))
    (send *ri* :move-to target :frame-id "/map" :no-wait nil)))

(defun lookup-transform (from to &key
                              (stamp (ros::time 0)))
  (setq *tfl* (instance ros::transform-listener :init))
  (when (null (send *tfl* :wait-for-transform from to stamp 2))
    (return-from lookup-transform nil))
  (send *tfl* :lookup-transform from to stamp))

(defun draw (coord)
  (send (send coord :copy-worldcoords) :draw-on :flush t
        :size 100 :width 5 :color (float-vector 1 0 0 )))

(defun start-grasp (&optional (real nil))
  (when real
    (send *ri* :start-grasp)
    (send *ri* :wait-interpolation)))

(defun stop-grasp (&optional (real nil))
  (when real
    (send *ri* :stop-grasp)
    (send *ri* :wait-interpolation)))

(defun ir2ri (&optional (time 10000))
  (send *ri* :angle-vector (send *robot* :angle-vector) time)
  (send *ri* :wait-interpolation))

(defun reset (&optional (real nil))
  (send *robot* :reset-pose)
  (when real
    (ir2ri 8000)))

(defun reset-go (&optional (real nil))
  (send *robot* :angle-vector (float-vector 0.0 0.0 9.99999 -0.153481 -89.9543 -89.9547 0.024563 -0.000336 0.0))
  (when real
    (ir2ri 8000)))

(defun find-stop-button-pose (&optional (real nil))
  (reset real)
  (send *robot* :rarm-torso-z :joint-angle 250)
  (send *robot* :head :neck-p :joint-angle -10)
  (when real
    (ir2ri 5000)
    ))

(defun get-yakan-button ()
  (setq target-coords nil)
  (when (null target-coords)
    (setq target-coords (lookup-transform "base_link" "button1")))
  (when target-coords
    (draw target-coords)
    (format t "~A~%" target-coords)
    (float-vector
     (elt (send target-coords :worldpos) 0)
     (elt (send target-coords :worldpos) 1)
     (elt (send target-coords :worldpos) 2)
     )))

(defun solve-ik (target grasp-type &optional (fl-vec (float-vector 0 0 0)) (real nil))
  (let (target-coords)
    (when (null target)
      (format t "cannot find target~%")
      (return-from solve-ik nil))
    (when (string= grasp-type "side")
      (setq target-coords
            (send (make-coords :pos target)
                  :rotate (deg2rad -90) :z))
      (send *robot* :inverse-kinematics
            ;;(send (send *robot* :copy-worldcoords))
            (send (send target-coords :copy-worldcoords) :translate
                  fl-vec :world)
            :ik-group :rarm-whole-body
            :rotation-axis t
            :revert-if-fail nil
            :look-at-target t :debug-view t)
      (draw target-coords)
      (send *irtviewer* :draw-objects)
      (when real
        (ir2ri 8000)))
    
    (when (string= grasp-type "top")
      (setq target-coords
            (send (make-coords :pos target)
                  :rotate (deg2rad 90) :y))
      (send *robot* :rarm :inverse-kinematics
            ;(send *robot* :copy-worldcoords)
            (send (send target-coords :copy-worldcoords) :translate
                  fl-vec :world)
            :ik-group :rarm-whole-body
            :rotation-axis t
            :revetif-fail nil
            :look-at-target t :debug-view t)
      (draw target-coords)
      (send *irtviewer* :draw-objects)
      (when real
        (ir2ri 8000)))
    ))

(defun get_bounding_box (color)
  (setq base-to-target-point nil)
  (setq target-coords nil)
  (setq msg (one-shot-subscribe
             (format nil "/HSI_color_filter/boxes_~A" color)
             jsk_recognition_msgs::BoundingBoxArray
             :timeout 10000))
  (when msg
    (setq distance-x nil)
    (setq min-x 1000000)
    (setq base-to-camera nil)
    (when (null base-to-camera)
      (setq base-to-camera (lookup-transform "/base_link" (send (car (send msg :boxes)) :header :frame_id))))

    (dolist (box (send msg :boxes))
      (setq camera-to-target-point
            (make-coords :pos (ros::tf-point->pos (send box :pose :position))))
      (setq base-to-target-point
            (send (send base-to-camera :copy-worldcoords) :transform
                  (send camera-to-target-point :copy-worldcoords)))
      (setq distance-x (elt (send base-to-target-point :worldpos) 0))
      (when (> min-x distance-x)
        (setq min-x distance-x)
        (setq target-coords base-to-target-point))))

  (setq *tfb* (instance ros::transform-broadcaster :init))
  (send *tfb* :send-transform target-coords
        "base_link" "target_coords_tf")
  (send *irtviewer :draw-objects)
  (x::window-main-one)
  (when target-coords
    (draw target-coords)
    (format t "~A~%" targeat-coords)
    (float-vector
     (elt (send target-coords :worldpos) 0)
     (elt (send target-coords :worldpos) 1)
     (elt (send target-coords :worldpos) 2)
     )))

(ros::roseus "agent")

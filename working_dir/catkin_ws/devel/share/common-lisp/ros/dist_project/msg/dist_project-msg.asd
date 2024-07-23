
(cl:in-package :asdf)

(defsystem "dist_project-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "robot_data" :depends-on ("_package_robot_data"))
    (:file "_package_robot_data" :depends-on ("_package"))
    (:file "uwb_data" :depends-on ("_package_uwb_data"))
    (:file "_package_uwb_data" :depends-on ("_package"))
  ))
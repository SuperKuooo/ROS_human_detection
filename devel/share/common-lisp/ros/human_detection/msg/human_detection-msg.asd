
(cl:in-package :asdf)

(defsystem "human_detection-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "bounding_box" :depends-on ("_package_bounding_box"))
    (:file "_package_bounding_box" :depends-on ("_package"))
  ))
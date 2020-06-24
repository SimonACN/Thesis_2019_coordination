
(cl:in-package :asdf)

(defsystem "drone_code-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "coord" :depends-on ("_package_coord"))
    (:file "_package_coord" :depends-on ("_package"))
  ))

(cl:in-package :asdf)

(defsystem "ardrone_application_node-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "serialized_ardrone_command_part" :depends-on ("_package_serialized_ardrone_command_part"))
    (:file "_package_serialized_ardrone_command_part" :depends-on ("_package"))
    (:file "serialized_ardrone_command" :depends-on ("_package_serialized_ardrone_command"))
    (:file "_package_serialized_ardrone_command" :depends-on ("_package"))
  ))
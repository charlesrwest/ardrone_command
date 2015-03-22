
(cl:in-package :asdf)

(defsystem "ardrone_command-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "serialized_ardrone_command_part" :depends-on ("_package_serialized_ardrone_command_part"))
    (:file "_package_serialized_ardrone_command_part" :depends-on ("_package"))
    (:file "test" :depends-on ("_package_test"))
    (:file "_package_test" :depends-on ("_package"))
    (:file "serialized_ardrone_command" :depends-on ("_package_serialized_ardrone_command"))
    (:file "_package_serialized_ardrone_command" :depends-on ("_package"))
  ))
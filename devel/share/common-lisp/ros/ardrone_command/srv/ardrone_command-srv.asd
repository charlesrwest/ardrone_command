
(cl:in-package :asdf)

(defsystem "ardrone_command-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :ardrone_command-msg
)
  :components ((:file "_package")
    (:file "commandInterface" :depends-on ("_package_commandInterface"))
    (:file "_package_commandInterface" :depends-on ("_package"))
  ))
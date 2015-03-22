; Auto-generated. Do not edit!


(cl:in-package ardrone_command-msg)


;//! \htmlinclude serialized_ardrone_command.msg.html

(cl:defclass <serialized_ardrone_command> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type ardrone_command-msg:serialized_ardrone_command_part
    :initform (cl:make-instance 'ardrone_command-msg:serialized_ardrone_command_part))
   (subcommands
    :reader subcommands
    :initarg :subcommands
    :type (cl:vector ardrone_command-msg:serialized_ardrone_command_part)
   :initform (cl:make-array 0 :element-type 'ardrone_command-msg:serialized_ardrone_command_part :initial-element (cl:make-instance 'ardrone_command-msg:serialized_ardrone_command_part))))
)

(cl:defclass serialized_ardrone_command (<serialized_ardrone_command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <serialized_ardrone_command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'serialized_ardrone_command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ardrone_command-msg:<serialized_ardrone_command> is deprecated: use ardrone_command-msg:serialized_ardrone_command instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <serialized_ardrone_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_command-msg:command-val is deprecated.  Use ardrone_command-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'subcommands-val :lambda-list '(m))
(cl:defmethod subcommands-val ((m <serialized_ardrone_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_command-msg:subcommands-val is deprecated.  Use ardrone_command-msg:subcommands instead.")
  (subcommands m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <serialized_ardrone_command>) ostream)
  "Serializes a message object of type '<serialized_ardrone_command>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'command) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'subcommands))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'subcommands))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <serialized_ardrone_command>) istream)
  "Deserializes a message object of type '<serialized_ardrone_command>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'command) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'subcommands) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'subcommands)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ardrone_command-msg:serialized_ardrone_command_part))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<serialized_ardrone_command>)))
  "Returns string type for a message object of type '<serialized_ardrone_command>"
  "ardrone_command/serialized_ardrone_command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'serialized_ardrone_command)))
  "Returns string type for a message object of type 'serialized_ardrone_command"
  "ardrone_command/serialized_ardrone_command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<serialized_ardrone_command>)))
  "Returns md5sum for a message object of type '<serialized_ardrone_command>"
  "ffbcc12c7c708ba17b5a85205ec7941d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'serialized_ardrone_command)))
  "Returns md5sum for a message object of type 'serialized_ardrone_command"
  "ffbcc12c7c708ba17b5a85205ec7941d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<serialized_ardrone_command>)))
  "Returns full string definition for message of type '<serialized_ardrone_command>"
  (cl:format cl:nil "#ROS message format doesn't support recursive definitions (a message type cannot contain more messages of the same type), so commands will have to be decomposed when they are serialized and will not be exactly the same on the other side~%serialized_ardrone_command_part command~%serialized_ardrone_command_part[] subcommands ~%~%~%================================================================================~%MSG: ardrone_command/serialized_ardrone_command_part~%uint32 type~%string[] strings~%float64[] doubles~%int64[] integers~%uint32[] flightAnimations~%uint32[] ledAnimations~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'serialized_ardrone_command)))
  "Returns full string definition for message of type 'serialized_ardrone_command"
  (cl:format cl:nil "#ROS message format doesn't support recursive definitions (a message type cannot contain more messages of the same type), so commands will have to be decomposed when they are serialized and will not be exactly the same on the other side~%serialized_ardrone_command_part command~%serialized_ardrone_command_part[] subcommands ~%~%~%================================================================================~%MSG: ardrone_command/serialized_ardrone_command_part~%uint32 type~%string[] strings~%float64[] doubles~%int64[] integers~%uint32[] flightAnimations~%uint32[] ledAnimations~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <serialized_ardrone_command>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'command))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'subcommands) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <serialized_ardrone_command>))
  "Converts a ROS message object to a list"
  (cl:list 'serialized_ardrone_command
    (cl:cons ':command (command msg))
    (cl:cons ':subcommands (subcommands msg))
))

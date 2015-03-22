; Auto-generated. Do not edit!


(cl:in-package ardrone_command-srv)


;//! \htmlinclude commandInterface-request.msg.html

(cl:defclass <commandInterface-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type ardrone_command-msg:serialized_ardrone_command
    :initform (cl:make-instance 'ardrone_command-msg:serialized_ardrone_command)))
)

(cl:defclass commandInterface-request (<commandInterface-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <commandInterface-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'commandInterface-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ardrone_command-srv:<commandInterface-request> is deprecated: use ardrone_command-srv:commandInterface-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <commandInterface-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_command-srv:command-val is deprecated.  Use ardrone_command-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <commandInterface-request>) ostream)
  "Serializes a message object of type '<commandInterface-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'command) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <commandInterface-request>) istream)
  "Deserializes a message object of type '<commandInterface-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'command) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<commandInterface-request>)))
  "Returns string type for a service object of type '<commandInterface-request>"
  "ardrone_command/commandInterfaceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'commandInterface-request)))
  "Returns string type for a service object of type 'commandInterface-request"
  "ardrone_command/commandInterfaceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<commandInterface-request>)))
  "Returns md5sum for a message object of type '<commandInterface-request>"
  "b3948502baec24ce327093a6cf3cf6ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'commandInterface-request)))
  "Returns md5sum for a message object of type 'commandInterface-request"
  "b3948502baec24ce327093a6cf3cf6ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<commandInterface-request>)))
  "Returns full string definition for message of type '<commandInterface-request>"
  (cl:format cl:nil "serialized_ardrone_command command~%~%================================================================================~%MSG: ardrone_command/serialized_ardrone_command~%#ROS message format doesn't support recursive definitions (a message type cannot contain more messages of the same type), so commands will have to be decomposed when they are serialized and will not be exactly the same on the other side~%serialized_ardrone_command_part command~%serialized_ardrone_command_part[] subcommands ~%~%~%================================================================================~%MSG: ardrone_command/serialized_ardrone_command_part~%uint32 type~%string[] strings~%float64[] doubles~%int64[] integers~%uint32[] flightAnimations~%uint32[] ledAnimations~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'commandInterface-request)))
  "Returns full string definition for message of type 'commandInterface-request"
  (cl:format cl:nil "serialized_ardrone_command command~%~%================================================================================~%MSG: ardrone_command/serialized_ardrone_command~%#ROS message format doesn't support recursive definitions (a message type cannot contain more messages of the same type), so commands will have to be decomposed when they are serialized and will not be exactly the same on the other side~%serialized_ardrone_command_part command~%serialized_ardrone_command_part[] subcommands ~%~%~%================================================================================~%MSG: ardrone_command/serialized_ardrone_command_part~%uint32 type~%string[] strings~%float64[] doubles~%int64[] integers~%uint32[] flightAnimations~%uint32[] ledAnimations~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <commandInterface-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <commandInterface-request>))
  "Converts a ROS message object to a list"
  (cl:list 'commandInterface-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude commandInterface-response.msg.html

(cl:defclass <commandInterface-response> (roslisp-msg-protocol:ros-message)
  ((received
    :reader received
    :initarg :received
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass commandInterface-response (<commandInterface-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <commandInterface-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'commandInterface-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ardrone_command-srv:<commandInterface-response> is deprecated: use ardrone_command-srv:commandInterface-response instead.")))

(cl:ensure-generic-function 'received-val :lambda-list '(m))
(cl:defmethod received-val ((m <commandInterface-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ardrone_command-srv:received-val is deprecated.  Use ardrone_command-srv:received instead.")
  (received m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <commandInterface-response>) ostream)
  "Serializes a message object of type '<commandInterface-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'received) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <commandInterface-response>) istream)
  "Deserializes a message object of type '<commandInterface-response>"
    (cl:setf (cl:slot-value msg 'received) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<commandInterface-response>)))
  "Returns string type for a service object of type '<commandInterface-response>"
  "ardrone_command/commandInterfaceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'commandInterface-response)))
  "Returns string type for a service object of type 'commandInterface-response"
  "ardrone_command/commandInterfaceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<commandInterface-response>)))
  "Returns md5sum for a message object of type '<commandInterface-response>"
  "b3948502baec24ce327093a6cf3cf6ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'commandInterface-response)))
  "Returns md5sum for a message object of type 'commandInterface-response"
  "b3948502baec24ce327093a6cf3cf6ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<commandInterface-response>)))
  "Returns full string definition for message of type '<commandInterface-response>"
  (cl:format cl:nil "bool received~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'commandInterface-response)))
  "Returns full string definition for message of type 'commandInterface-response"
  (cl:format cl:nil "bool received~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <commandInterface-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <commandInterface-response>))
  "Converts a ROS message object to a list"
  (cl:list 'commandInterface-response
    (cl:cons ':received (received msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'commandInterface)))
  'commandInterface-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'commandInterface)))
  'commandInterface-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'commandInterface)))
  "Returns string type for a service object of type '<commandInterface>"
  "ardrone_command/commandInterface")
#ROS message format doesn't support recursive definitions (a message type cannot contain more messages of the same type), so commands will have to be decomposed when they are serialized and will not be exactly the same on the other side
serialized_ardrone_command_part command
serialized_ardrone_command_part[] subcommands 


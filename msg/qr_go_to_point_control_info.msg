#This message is published when the drone is going to a particular point using QR code coordinates.

time time_stamp
float64[3] target_point_camera_xyz #The estimated position of the target point in the AR drone's camera coordinate system
float64[3] target_point_local_xyz #The estimated position of the target point in the AR drone movement system coordinate system
float64 estimated_distance_to_target

#The control algorithm has two different PID value sets that it uses.  One is optimized for maintaining position when the drone is close to the point (near) and the other is optimized for getting the drone close to the point without gaining too much speed (far).  The drone switches between these sets based on how far it thinks it is from the target.

uint32 mode #0 = near, 1 = far
float64 qr_x_axis_I_term #The I part of the PID control for the quadrotor x axis
float64 qr_y_axis_I_term #The I part of the PID control for the quadrotor y axis

float64[3] qr_xyz_throttle #The final values sent to the AR drone for desired speed in XYZ


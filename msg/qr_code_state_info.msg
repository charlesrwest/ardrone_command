#This message contains the info associated with seeing a QR code with the AR Drone an estimating position from it.  Multiple messages can be published from a single frame if there is more than one QR code (which can be detected by looking at the time stamps).

time time_stamp
string  qr_code_identifier
float64 qr_code_size #Dimension of one of the sides of the QR code (in meters)

float64[16] transform #4x4 transform of the drone associated with this QR code.  Column row format: x11, x21, x31, x41, x12, etc



